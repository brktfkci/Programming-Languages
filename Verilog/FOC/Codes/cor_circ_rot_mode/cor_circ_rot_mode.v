//=================================================================================================
// cordic_rot_foc module declaration
//=================================================================================================
module cordic
#(
// Generic List
    // Specifies IO_WIDTH of all input and output ports
    parameter IO_WIDTH = 18,

    // Specifies IO_WIDTH of all input and output ports
    parameter ITER_NUM = 15,

    // Specifies the width of multiplication result
    parameter ADD_WIDTH = 44,

    // Specifies the number of sys_clk_i delays required before the mas done signal is asserted.
    parameter CYCLE_NUM = 2	
)
 (
// Port List
    // System reset
    input reset_i,

    // System clock
    input sys_clk_i,

    // Control input specifies the start of cordic operation
    input start_i,

    // x co-ordinate input for cordic
    input signed [IO_WIDTH-1:0] x_i,

    // y co-ordinate input for cordic
    input signed [IO_WIDTH-1:0]  y_i,

    // angle input for cordic
    input signed [IO_WIDTH-1:0]  theta_i,

    // Control output specifies that the cordic has finished its operation
    output reg done_o,

    // X co-ordinate output of cordic after rotation
    output reg signed [IO_WIDTH-1:0] x_o,

    // Y co-ordinate output of cordic after rotation
    output reg signed [IO_WIDTH-1:0]  y_o,

     // Z co-ordinate output of cordic after rotation
    output reg signed [IO_WIDTH-1:0]  theta_o
);

//=================================================================================================
// Signal declarations
//=================================================================================================
// 90 degrees = 65536, 180 = 131072, 360 = 262144

// atan(2^0)  = 45      = 32768	= x8000
// atan(2^-1) = 26.57   = 19344 = x4B90
// atan(2^-2) = 14.04   = 10221	= x27ED
// atan(2^-3) = 7.125   = 5188 	= x1444
// atan(2^-4) = 3.576   = 2604	= xA2C
// atan(2^-5) = 1.79    = 1303	= x517
// atan(2^-6) = 0.893   = 652	= x28C
// atan(2^-7) = 0.448   = 326	= x146
// atan(2^-8) = 0.224   = 163	= xA3
// atan(2^-9) = 0.112   = 81	= x51
// atan(2^-10) = 0.056  = 41	= x29
// atan(2^-11) = 0.028  = 20	= x14
// atan(2^-12) = 0.014  = 10	= xA
// atan(2^-13) = 0.007  = 5		= x5
// atan(2^-14) = 0.0035 = 3		= x3
// atan(2^-15) = 0.0017 = 1		= x1

wire [IO_WIDTH-1:0]CORDIC_ANG[0:15];

assign CORDIC_ANG[0]   = 18'h08000;
assign CORDIC_ANG[1]   = 18'h04B90;
assign CORDIC_ANG[2]   = 18'h027ED;
assign CORDIC_ANG[3]   = 18'h01444;
assign CORDIC_ANG[4]   = 18'h00A2C;
assign CORDIC_ANG[5]   = 18'h00517;
assign CORDIC_ANG[6]   = 18'h0028C;
assign CORDIC_ANG[7]   = 18'h00146;
assign CORDIC_ANG[8]   = 18'h000A3;
assign CORDIC_ANG[9]   = 18'h00051;
assign CORDIC_ANG[10]  = 18'h00029;
assign CORDIC_ANG[11]  = 18'h00014;
assign CORDIC_ANG[12]  = 18'h0000A;
assign CORDIC_ANG[13]  = 18'h00005;
assign CORDIC_ANG[14]  = 18'h00003;
assign CORDIC_ANG[15]  = 18'h00001;

localparam [1:0] IDLE = 2'd0, ROT  = 2'd1, SCALE_X = 2'd2, SCALE_Y = 2'd3;


reg [IO_WIDTH-1:0] x_o_next;
reg [IO_WIDTH-1:0] y_o_next;				
reg [IO_WIDTH-1:0] theta_o_next;	
reg done_o_next;			
reg [1:0] state, state_next;
reg [3:0] index, index_next;
reg dir_rot, dir_rot_next;
reg [IO_WIDTH-1:0] xi_rot, xi_rot_next;
reg [IO_WIDTH-1:0] yi_rot, yi_rot_next;
reg [IO_WIDTH-1:0] theta_rot, theta_rot_next;

reg [IO_WIDTH-1:0] x_out;
reg [IO_WIDTH-1:0] y_out;
reg [IO_WIDTH-1:0] theta_out;

wire [IO_WIDTH:0] dir_1_dif_xi_rot;
wire [IO_WIDTH:0] dir_2_sum_xi_rot;
wire [IO_WIDTH:0] dir_1_sum_yi_rot;
wire [IO_WIDTH:0] dir_2_dif_yi_rot;
wire [IO_WIDTH:0] dir_1_dif_theta_rot;
wire [IO_WIDTH:0] dir_2_sum_theta_rot;


reg [IO_WIDTH-1:0] mult0_input;
reg [IO_WIDTH-1:0] mult1_input;
reg mas_en;
wire mas_done;
wire [IO_WIDTH-1:0] scale_cons;
wire [ADD_WIDTH-1:0] product;

assign scale_cons = 18'h0026E;



//=================================================================================================
// Asynchronous blocks
//=================================================================================================
assign dir_1_dif_xi_rot = $signed(xi_rot_next) - $signed($signed(yi_rot_next) >>> (index_next-1));
assign dir_1_sum_yi_rot = $signed(yi_rot_next) + $signed($signed(xi_rot_next) >>> (index_next-1));
assign dir_1_dif_theta_rot = ~reset_i ? theta_rot_next - CORDIC_ANG[index_next-1] : 0;

assign dir_2_sum_xi_rot = $signed(xi_rot_next) + $signed($signed(yi_rot_next) >>> (index_next-1));
assign dir_2_dif_yi_rot = $signed(yi_rot_next) - $signed($signed(xi_rot_next) >>> (index_next-1));
assign dir_2_sum_theta_rot = ~reset_i ? theta_rot_next + CORDIC_ANG[index_next-1] : 0;

//------------------------------------------------------------------------
// Name       : MICRO_ROT_PROC
// Description: Process to compute micro rotations
//------------------------------------------------------------------------
	always@(posedge sys_clk_i) begin
		if(dir_rot_next == 1'b1) begin
			x_out     <= #1 dir_1_dif_xi_rot[IO_WIDTH-1:0];
			y_out     <= #1 dir_1_sum_yi_rot[IO_WIDTH-1:0];
            theta_out <= #1 dir_1_dif_theta_rot[IO_WIDTH-1:0];
		end 
		else begin
            x_out     <= #1 dir_2_sum_xi_rot[IO_WIDTH-1:0];
			y_out     <= #1 dir_2_dif_yi_rot[IO_WIDTH-1:0];
            theta_out <= #1 dir_2_sum_theta_rot[IO_WIDTH-1:0];
		end
	end
//------------------------------------------------------------------------
// Name       : CORDIC_FSM_PROC
// Description: FSM implements cordic operations
//------------------------------------------------------------------------
	always @(posedge sys_clk_i) begin
		xi_rot 	   <= #1 xi_rot_next;
		yi_rot 	   <= #1 yi_rot_next;
		theta_rot  <= #1 theta_rot_next; 
		dir_rot    <= #1 dir_rot_next;        
		state  	   <= #1 state_next;               
		index	   <= #1 index_next;
		done_o     <= #1 done_o_next;
	end
	
	always @(*) begin :CORDIC_FSM_PROC
		xi_rot_next    = xi_rot;	 
		yi_rot_next    = yi_rot;
		theta_rot_next = theta_rot;
		dir_rot_next   = dir_rot;      
		state_next     = state; 	          
		index_next     = index;
        done_o_next    = 1'b0;
		mult0_input    = {IO_WIDTH{1'b0}};
		mult1_input    = {IO_WIDTH{1'b0}};
		mas_en         = 0;
		if (reset_i == 1'b1) begin
			xi_rot_next	   = {IO_WIDTH{1'b0}};
			yi_rot_next	   = {IO_WIDTH{1'b0}}; 
         theta_rot_next = {IO_WIDTH{1'b0}};       
			state_next	   = IDLE;
         index_next	   = 0;
			done_o_next    = 1'b0;
			mult0_input    = {IO_WIDTH{1'b0}};
			mult1_input    = {IO_WIDTH{1'b0}};	
			if($signed(theta_i) > 0) begin
				dir_rot_next   = 1'b0;
			end else begin
				dir_rot_next   = 1'b1;
			end
		end 
		else begin
			case (state)
//------------------
// IDLE state
//------------------
			IDLE :
			begin
				if(start_i == 1'b1) begin
					xi_rot_next	= x_i;
					yi_rot_next	= y_i;
					theta_rot_next = theta_i;
					state_next	= ROT;
					index_next	 = 1;					
				end
			end
//------------------
// ROT state
//------------------
			ROT : 
			begin
				if(index == ITER_NUM) begin
					mas_en = 1;
					mult0_input = xi_rot;
					mult1_input = scale_cons;
					state_next  = SCALE_X;
				end 
				else begin
					xi_rot_next = x_out;
					yi_rot_next = y_out;
               theta_rot_next = theta_out;
					if (index < ITER_NUM) begin
						if($signed (theta_rot_next) > 0) begin
							dir_rot_next	= 1'b1;
						end 
						else begin
							dir_rot_next	= 1'b0;
						end
						index_next = index + 1;
					end
				end
			end
			SCALE_X: 
			begin
				if(mas_done) begin
					mas_en = 1;
					mult0_input = yi_rot;
					mult1_input = scale_cons;             
					state_next  = SCALE_Y;
					dir_rot_next   = 1'b0; 
					index_next	   = 1;		
				end
			end
			SCALE_Y: 
			begin
				if(mas_done) begin
					done_o_next = 1'b1;	
					state_next  = IDLE;
					dir_rot_next   = 1'b0; 
					index_next	   = 1;		
				end
			end            
			endcase
		end
	end



//------------------------------------------------------------------------
// Name       : OUTPUT_PROC
// Description: Assigns outputs of cordic
//------------------------------------------------------------------------
    always@(posedge sys_clk_i) begin
		x_o     <= #1 x_o_next;
		y_o     <= #1 y_o_next;
		theta_o <= #1 theta_o_next;
	end
	
    always@(*) begin: OUTPUT_PROC
		x_o_next     = x_o;
		y_o_next     = y_o;
		theta_o_next = theta_o;
        if (reset_i == 1'b1) begin
            x_o_next     = {IO_WIDTH{1'b0}};
			y_o_next     = {IO_WIDTH{1'b0}};
			theta_o_next = {IO_WIDTH{1'b0}};
		end 
		else begin
			if(state == SCALE_X &&  mas_done == 1) begin
                x_o_next = product [IO_WIDTH +9 :10];
			end 
         else if(state == SCALE_Y &&  mas_done == 1) begin
                y_o_next = product [IO_WIDTH +9 :10];
         end
			else begin
				theta_o_next = theta_rot;
			end
        end
    end

//=================================================================================================
// module Instantiations
//=================================================================================================
//------------------------------------------------------------------------
// Name       : MAS_INST
// Description: 18x18 MAS block
//------------------------------------------------------------------------

mass_block
#(
    .IO_WIDTH (IO_WIDTH),
    .ADD_WIDTH (ADD_WIDTH),
    .CYCLE_NUM (CYCLE_NUM)
)
mass_block_cor_circ_vect(
    .reset_i                           (reset_i),
    .sys_clk_i                         (sys_clk_i),
	.mas_en_i                          (mas_en),
    .sub_i                             (1'b0),
    .mul_a_i                           (mult0_input),
    .mul_b_i                           (mult1_input),
    .add_c_i                           (44'b0),
    .product_o                         (product),
    .mas_done_o                        (mas_done)
);


endmodule