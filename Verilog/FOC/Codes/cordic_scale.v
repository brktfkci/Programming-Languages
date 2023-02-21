//=================================================================================================
// cordic_scale module declaration
//=================================================================================================
module cordic_scale
#(
// Generic List
    // Specifies width of all input and output ports
    parameter g_STD_IO_WIDTH = 18,

	// width of ADD_C input of MAS
	parameter g_ADD_C_WIDTH = 44

)
 (
// Port List
    // System reset
    input reset_i,

    // System clock
    input sys_clk_i,

    // Control input specifies the start of cordic operation
    input start_i,

    // x co-ordinate input for cordic scaling
    input signed [g_STD_IO_WIDTH-1:0] x_i,

    // y co-ordinate input for cordic scaling
    input signed [g_STD_IO_WIDTH-1:0] y_i,

	// mas result
	input signed [g_ADD_C_WIDTH-1:0] mas_product_i,

	// done signal from mas
	input mas_done_i,

    // Control output specifies that the cordic scaling has finished its operation
    output reg done_o,

    // X co-ordinate output of scaling
    output reg [g_STD_IO_WIDTH-1:0] x_o,

    // Y co-ordinate output of scaling
    output reg [g_STD_IO_WIDTH-1:0] y_o,

	// mul0 input for mas
	output reg [g_STD_IO_WIDTH-1:0] mas_mul_a_o,

	// mul1 input for mas
    output wire [g_STD_IO_WIDTH-1:0] mas_mul_b_o,

	// add c input for mas
    output wire [g_ADD_C_WIDTH-1:0] mas_add_c_o,

	// mas enable signal
    output reg mas_en_o
);

//=================================================================================================
// Register and wire declarations
//=================================================================================================
localparam [1:0] IDLE = 2'd0,
				SCALE_X = 2'd1,
				SCALE_Y = 2'd2;

reg done_o_next;
reg [g_STD_IO_WIDTH-1:0] x_o_next;
reg [g_STD_IO_WIDTH-1:0] y_o_next;
reg [g_STD_IO_WIDTH-1:0] mas_mul_a_o_next;
reg [g_STD_IO_WIDTH-1:0] mas_mul_a1_o;
reg [g_STD_IO_WIDTH-1:0] mas_mul_a1_o_next;
reg mas_en_o_next;
reg [1:0] s_state ;
reg [1:0] s_state_next ;

//=================================================================================================
// Top level output port assignments
//=================================================================================================
assign mas_mul_b_o         = 18'h0026E;
assign mas_add_c_o         = {g_ADD_C_WIDTH{1'b0}};

//=================================================================================================
// Synchronous blocks
//=================================================================================================
//--------------------------------------------------------------------------
// Name       : CORDIC_SCALE_FSM_PROC
// Description: FSM implements the cordic operations
//--------------------------------------------------------------------------
	always@(posedge sys_clk_i) begin
		s_state 	 <= #1 s_state_next; 	
		mas_en_o 	 <= #1 mas_en_o_next; 	
		done_o 		 <= #1 done_o_next; 		 
		mas_mul_a_o	 <= #1 mas_mul_a_o_next;	 
		mas_mul_a1_o <= #1 mas_mul_a1_o_next;	 
		x_o 		 <= #1 x_o_next; 		 
		y_o 		 <= #1 y_o_next; 		 
	end
	
	always@(*) begin:CORDIC_SCALE_FSM_PROC
		s_state_next      = s_state;
		mas_en_o_next     = mas_en_o;
		done_o_next	      = done_o;
		mas_mul_a_o_next  = mas_mul_a_o;
		mas_mul_a1_o_next = mas_mul_a1_o;
		x_o_next	      = x_o;
		y_o_next	      = y_o;
		if ( reset_i == 1'b0) begin
			s_state_next	 = IDLE;
			mas_en_o_next	 = 1'b0;
			done_o_next	     = 1'b0;
			mas_mul_a_o_next = {g_STD_IO_WIDTH{1'b0}};
			x_o_next	     = {g_STD_IO_WIDTH{1'b0}};
			y_o_next	     = {g_STD_IO_WIDTH{1'b0}};
		end 
		else begin
			case (s_state)
//-------------------
// IDLE state
//-------------------
			IDLE : 
			begin
				done_o_next	= 1'b0;
				mas_mul_a_o_next	= x_i;
				mas_mul_a1_o_next	= y_i;
				if (start_i == 1'b1) begin
					s_state_next	= SCALE_X;
				    mas_en_o_next	= 1'b1;
                end 
				else begin
				    s_state_next	= IDLE;
				    mas_en_o_next	= 1'b0;
                end
            end
//-------------------
// SCALE_X state
//-------------------
			SCALE_X : 
			begin
				if(mas_done_i == 1'b1) begin
					s_state_next  = SCALE_Y;
				    mas_en_o_next = 1'b1;
				    x_o_next      = mas_product_i[g_STD_IO_WIDTH+8 : 9];
                end 
				else begin
				    s_state_next	 = SCALE_X;
				    mas_mul_a_o_next = mas_mul_a1_o;
				    mas_en_o_next	 = 1'b0;
                end
			end
//-------------------
// SCALE_Y state
//-------------------
			SCALE_Y : 
			begin
				mas_en_o_next	= 1'b0;
				if(mas_done_i == 1'b1) begin
					s_state_next  = IDLE;
					mas_en_o_next = 1'b0;
				    y_o_next	  = mas_product_i[g_STD_IO_WIDTH+8 : 9];
					done_o_next	  = 1'b1;
                end 
				else begin
				    s_state_next	= SCALE_Y;
				end
            end
//-------------------
// OTHERS state
//-------------------
			default : s_state_next	= IDLE;
			endcase
		end
	end

endmodule
