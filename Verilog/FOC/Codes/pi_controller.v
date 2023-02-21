//=================================================================================================
// pi_controller module declaration
//=================================================================================================
module PI_CONTROLLER
#(
    // Specifies the number of multi-cycle path delays introduced
    parameter g_NO_MCYCLE_PATH = 2

)
 (
// Port List
    // System reset
    input reset_i,

    // System clock
    input sys_clk_i,

    // Control input specifies the start of pi controller operation
    input start_i,

    // Control input specifies whether the pi controller has to be initialized
    input pi_en_i,

	// Resets the accumulator of PI
    input clear_buffer_i,

    // Data input specifies the reference input of pi controller
    input signed [17:0] ref_input_i,

    // Data input specifies the actual input of pi controller
    input signed [17:0] act_input_i,

    // Control input specifies the kp value
    input signed [17:0] kp_i,

    // Control input specifies the ki value
    input signed [17:0] ki_i,

    // Control input specifies the maximum limit for the pi controller output
    input signed [17:0] ymax_i,

    // Control input specifies the minimum limit for the pi controller output
    input signed [17:0] ymin_i,

    // Control input specifies the initialization value when pi_en_i is 1'b1
    input signed [17:0] init_i,

    // Control output specifies that the pi_controller has finished its operation
    output reg done_o,

    // Data output specifies the pi controller output valua
    output wire [17:0] output_y_o
);

//=================================================================================================
// Signal declarations
//=================================================================================================
// Specifies the de-scaling kp value
parameter  C_KP_Factor = 10;

// Specifies the de-scaling ki value
parameter  C_KI_Factor = 16;

parameter g_ADD_C_WIDTH = 44;
parameter [2:0] IDLE   = 3'd0,
                P_OUT  = 3'd1,
                I_OUT  = 3'd2,
                LIMIT  = 3'd3,
                OUTPUT = 3'd4;

reg done_o_next;
reg s_mas_en;
reg s_mas_en_next;
wire s_mas_done;
reg  [2:0] s_state;
reg  [2:0] s_state_next;
reg  [17:0]   s_mult0_input;
reg  [17:0]   s_mult0_input_next;
reg  [17:0]   s_mult1_input;
reg  [17:0]   s_mult1_input_next;
reg  [17 : 0] s_i_init_ds;
reg  [17 : 0] s_i_init_ds_next;
reg  [g_ADD_C_WIDTH-1  : 0] s_acc;
reg  [g_ADD_C_WIDTH-1  : 0] s_acc_next;
reg  [g_ADD_C_WIDTH-1  : 0] s_pout;
reg  [g_ADD_C_WIDTH-1  : 0] s_pout_next;
reg  [g_ADD_C_WIDTH-1  : 0] s_iout;
reg  [g_ADD_C_WIDTH-1  : 0] s_iout_next;
reg  [g_ADD_C_WIDTH-1  : 0] s_inew;
reg  [g_ADD_C_WIDTH-1  : 0] s_inew_next;
reg  [g_ADD_C_WIDTH-1  : 0] s_add_cin;
reg  [g_ADD_C_WIDTH-1  : 0] s_add_cin_next;
wire [17 : 0] s_error;
wire [g_ADD_C_WIDTH-1  : 0] s_product;
wire [g_ADD_C_WIDTH-1  : 0] s_ymax;
wire [g_ADD_C_WIDTH-1  : 0] s_ymin;
wire [g_ADD_C_WIDTH-1  : 0] s_init;
wire [g_ADD_C_WIDTH-1  : 0] s_i_init;
wire [g_ADD_C_WIDTH-1  : 0] s_pout_ds;
wire [g_ADD_C_WIDTH-1  : 0] s_iout_ds;
wire [g_ADD_C_WIDTH-1  : 0] s_piout;
wire [g_ADD_C_WIDTH-1  : 0] s_init_ext;
reg  [g_ADD_C_WIDTH-1  : 0] s_output_y;
reg  [g_ADD_C_WIDTH-1  : 0] s_output_y_next;
wire [g_ADD_C_WIDTH  : 0] s_sum;
wire [18 : 0] s_diff;

//=================================================================================================
// Top level output port assignments
//=================================================================================================
assign output_y_o	= s_output_y[17 : 0];

//=================================================================================================
// Asynchronous blocks
//=================================================================================================
assign s_error		= ref_input_i - act_input_i;
assign s_ymax       = {{(g_ADD_C_WIDTH-18){ymax_i[17]}},ymax_i};
assign s_ymin       = {{(g_ADD_C_WIDTH-18){ymin_i[17]}},ymin_i};
assign s_init       = {{(g_ADD_C_WIDTH- 18-C_KI_Factor){s_i_init_ds[17]}},s_i_init_ds,{C_KI_Factor{1'b0}}};
assign s_pout_ds    = {{(C_KP_Factor){s_pout[g_ADD_C_WIDTH-1]}},s_pout[g_ADD_C_WIDTH-1 : C_KP_Factor]};
assign s_iout_ds    = {{(C_KI_Factor){s_iout[g_ADD_C_WIDTH-1]}},s_iout[g_ADD_C_WIDTH-1 : C_KI_Factor]};
assign s_init_ext   = {{(g_ADD_C_WIDTH-18){s_init[17]}},s_init};
assign s_sum        = (s_pout_ds + s_iout_ds);
assign s_piout		= (pi_en_i === 1'b1) ? s_sum[g_ADD_C_WIDTH-1 :0]  : s_init_ext;
assign s_diff       = (init_i - s_pout_ds[17 : 0]);

//------------------------------------------------------------------------
// Name       : IOUT_SELECT_PROC
// Description: FSM implements the pi controller operations
//------------------------------------------------------------------------
    always@(posedge sys_clk_i)begin
		s_iout <= #1 s_iout_next;
	end
	
    always@(*) begin:IOUT_SELECT_PROC
		s_iout_next = s_iout;
		if(pi_en_i == 1'b1)
			s_iout_next	= s_inew;
		else
			s_iout_next = s_i_init;
	end

//------------------------------------------------------------------------
// Name       : PI_CONT_FSM_PROC
// Description: FSM implements the pi controller operations
//------------------------------------------------------------------------
    always@(posedge sys_clk_i) begin
		s_state         <= #1 s_state_next;
		s_mas_en     	<= #1 s_mas_en_next;
		done_o          <= #1 done_o_next;
		s_acc			<= #1 s_acc_next;
		s_pout			<= #1 s_pout_next;
		s_inew			<= #1 s_inew_next;
		s_mult0_input   <= #1 s_mult0_input_next;
		s_mult1_input   <= #1 s_mult1_input_next;
		s_add_cin       <= #1 s_add_cin_next;
		s_output_y		<= #1 s_output_y_next;
		s_i_init_ds		<= #1 s_i_init_ds_next;
	end
	
    always@(*) begin :PI_CONT_FSM_PROC
        s_state_next  = s_state;   
		s_mas_en_next = s_mas_en;	
		done_o_next	= done_o;   
		s_acc_next  = s_acc;	
		s_pout_next	= s_pout;	
		s_inew_next	= s_inew;	
		s_mult0_input_next = s_mult0_input;   
		s_mult1_input_next = s_mult1_input;   
		s_add_cin_next     = s_add_cin;   
		s_output_y_next	   = s_output_y;	
		s_i_init_ds_next   = s_i_init_ds;	
		if(reset_i == 1'b0) begin
            s_state_next	= IDLE;
            s_mas_en_next	= 1'b0;
            done_o_next	= 1'b0;
			s_acc_next	= {g_ADD_C_WIDTH{1'b0}};
			s_pout_next	= {g_ADD_C_WIDTH{1'b0}};
			s_inew_next	= {g_ADD_C_WIDTH{1'b0}};
			s_mult0_input_next	= {18{1'b0}};
            s_mult1_input_next	= {18{1'b0}};
            s_add_cin_next	= {g_ADD_C_WIDTH{1'b0}};
			s_output_y_next	= {g_ADD_C_WIDTH{1'b0}};
			s_i_init_ds_next	= {18{1'b0}};
		end
        else begin
            case (s_state)
//------------------
// IDLE state
//------------------
                IDLE :
				begin
					done_o_next	= 1'b0;
                    if (start_i == 1'b1) begin
                        s_state_next  = P_OUT;
                        s_mas_en_next = 1'b1;
                        s_mult0_input_next = s_error;
                        s_mult1_input_next = kp_i;
                        s_add_cin_next = {g_ADD_C_WIDTH{1'b0}};
					end
                    else
                        s_state_next = IDLE;
				end
//------------------
// P_OUT state
//------------------
                P_OUT :
				begin
                    if (s_mas_done == 1'b1) begin
						s_pout_next	= s_product;
                        s_mas_en_next = 1'b1;
						s_mult0_input_next = s_error;
						s_mult1_input_next = ki_i;
						s_add_cin_next	= s_acc;
                        s_state_next	= I_OUT;
					end
                    else begin
                        s_state_next	= P_OUT;
						s_mas_en_next	= 1'b0;
                    end
				end
//------------------
// I_OUT state
//------------------
                I_OUT :
				begin
					s_i_init_ds_next = s_diff[17 : 0];
                    if (s_mas_done == 1'b1) begin
						s_inew_next	= s_product;
                        s_state_next = LIMIT;
                        s_mas_en_next = 1'b0;
					end
                    else begin
                        s_state_next = I_OUT;
						s_mas_en_next = 1'b0;
					end
				end
//------------------
// LIMIT state
//------------------
                LIMIT :
				begin
                    s_state_next = OUTPUT;
					if ($signed(s_piout) < $signed(s_ymin))
						s_output_y_next	= s_ymin ;
					else if ($signed(s_piout) > $signed(s_ymax))
						s_output_y_next	= s_ymax ;
					else begin
						s_output_y_next	= s_piout;
						s_acc_next = s_iout;
					end

				end
//------------------
// OUTPUT state
//------------------
                OUTPUT :
				begin
                    s_state_next	= IDLE;
					done_o_next	= 1'b1;
				end
//------------------
// DEFAULT
//------------------
                default :
				begin
                    s_state_next	= IDLE;
					s_mas_en_next	= 1'b0;
					s_mult0_input_next	= {18{1'b0}};
                    s_mult1_input_next	= {18{1'b0}};
                    s_add_cin_next	= {g_ADD_C_WIDTH{1'b0}};
				end
            endcase
			if(clear_buffer_i == 1'b1)
				s_acc_next	= {g_ADD_C_WIDTH{1'b0}};
        end
    end


//=================================================================================================
// module Instantiations
//=================================================================================================
//------------------------------------------------------------------------
// Name       : MAS_PI_INST
// Description: 18x18 MAS block
//------------------------------------------------------------------------

mas_pi_cont
#(
    .g_STD_IO_WIDTH                    (18),
    .g_ADD_C_WIDTH                     (g_ADD_C_WIDTH),
    .g_NO_MCYCLE_PATH                  (g_NO_MCYCLE_PATH)
)
inst(
    .reset_i                           (reset_i),
    .sys_clk_i                         (sys_clk_i),
	.mas_en_i                          (s_mas_en),
    .sub_i                             (1'b0),
    .mul_a_i                           (s_mult0_input),
    .mul_b_i                           (s_mult1_input),
    .add_c_i                           (s_add_cin),
    .product_o                         (s_product),
    .mas_done_o                        (s_mas_done)
);

endmodule


