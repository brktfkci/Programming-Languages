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

    // Control output specifies that the cordic has finished its operation
    output reg done_o,

    // X co-ordinate output of cordic after rotation
    output reg signed [IO_WIDTH-1:0] x_o,

    // Y co-ordinate output of cordic after rotation
    output reg signed [IO_WIDTH-1:0]  y_o

     // Z co-ordinate output of cordic after rotation
);

localparam [1:0] IDLE = 2'd0, ROT  = 2'd1, SCALE_X = 2'd2, SCALE_Y = 2'd3;


reg [IO_WIDTH-1:0] tmp_x_i, tmp_y_i;
reg [IO_WIDTH-1:0] x_o_next;
reg [IO_WIDTH-1:0] y_o_next;				
reg done_o_next;			
reg [1:0] state, state_next;
reg flag, flag_next;
reg [3:0] index, index_next;
reg dir_rot, dir_rot_next;
reg [IO_WIDTH-1:0] xi_rot, xi_rot_next;
reg [IO_WIDTH-1:0] yi_rot, yi_rot_next;

reg [IO_WIDTH-1:0] x_out;
reg [IO_WIDTH-1:0] y_out;


wire [IO_WIDTH:0] dir_1_sum_xi_rot;
wire [IO_WIDTH:0] dir_1_sum_yi_rot;
wire [IO_WIDTH:0] dir_2_dif_xi_rot;
wire [IO_WIDTH:0] dir_2_dif_yi_rot;


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
assign dir_1_sum_xi_rot = $signed(xi_rot_next) + $signed($signed(yi_rot_next) >>> (index_next-1));
assign dir_1_sum_yi_rot = $signed(yi_rot_next) + $signed($signed(xi_rot_next) >>> (index_next-1));

assign dir_2_dif_xi_rot = $signed(xi_rot_next) - $signed($signed(yi_rot_next) >>> (index_next-1));
assign dir_2_dif_yi_rot = $signed(yi_rot_next) - $signed($signed(xi_rot_next) >>> (index_next-1));

//------------------------------------------------------------------------
// Name       : MICRO_ROT_PROC
// Description: Process to compute micro rotations
//------------------------------------------------------------------------
	always@(posedge sys_clk_i) begin
		if(dir_rot_next == 1'b1) begin
			x_out     <= #1 dir_1_sum_xi_rot[IO_WIDTH-1:0];
			y_out     <= #1 dir_1_sum_yi_rot[IO_WIDTH-1:0];
		end 
		else begin
            x_out     <= #1 dir_2_dif_xi_rot[IO_WIDTH-1:0];
			y_out     <= #1 dir_2_dif_yi_rot[IO_WIDTH-1:0];
		end
	end
//------------------------------------------------------------------------
// Name       : CORDIC_FSM_PROC
// Description: FSM implements cordic operations
//------------------------------------------------------------------------
	always @(posedge sys_clk_i) begin
		xi_rot 	   <= #1 xi_rot_next;
		yi_rot 	   <= #1 yi_rot_next;
		dir_rot    <= #1 dir_rot_next;        
		state  	   <= #1 state_next;               
		index	   <= #1 index_next;
		flag       <= #1 flag_next;		
		done_o     <= #1 done_o_next;
	end
	
	always @(*) begin :CORDIC_FSM_PROC
		xi_rot_next    = xi_rot;	 
		yi_rot_next    = yi_rot;
		dir_rot_next   = dir_rot;      
		state_next     = state; 	          
		index_next     = index;
		flag_next      = flag;
        done_o_next    = 1'b0;
		mult0_input    = {IO_WIDTH{1'b0}};
		mult1_input    = {IO_WIDTH{1'b0}};
		mas_en         = 0;
		if (reset_i == 1'b1) begin
			xi_rot_next	   = {IO_WIDTH{1'b0}};
			yi_rot_next	   = {IO_WIDTH{1'b0}}; 
			state_next	   = IDLE;
         	index_next	   = 0;
         	flag_next	   = 0;
			done_o_next    = 1'b0;
			mult0_input    = {IO_WIDTH{1'b0}};
			mult1_input    = {IO_WIDTH{1'b0}};	
			if($signed(y_i) < 0) begin
				dir_rot_next   = 1'b1;
			end else begin
				dir_rot_next   = 1'b0;
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
					case({y_i[IO_WIDTH-1], x_i[IO_WIDTH-1]})
					0: begin
						if(y_i > x_i) begin
							xi_rot_next	= y_i;
							yi_rot_next	= x_i;
						end else begin
							xi_rot_next	= x_i;
							yi_rot_next	= y_i;							
						end
					end
					1: begin
						tmp_x_i = ~x_i + 1;
						if(y_i > tmp_x_i) begin
							xi_rot_next	= y_i;
							yi_rot_next	= tmp_x_i;
						end else begin
							xi_rot_next	= tmp_x_i;
							yi_rot_next	= y_i;							
						end										
					end
					2: begin
						tmp_y_i = ~y_i + 1;
						if(tmp_y_i > x_i) begin
							xi_rot_next	= tmp_y_i;
							yi_rot_next	= x_i;
						end else begin
							xi_rot_next	= x_i;
							yi_rot_next	= tmp_y_i;							
						end						
					end
					3: begin
						tmp_x_i = ~x_i + 1;
						tmp_y_i = ~y_i + 1;
						if(tmp_y_i > tmp_x_i) begin
							xi_rot_next	= tmp_y_i;
							yi_rot_next	= tmp_x_i;
						end else begin
							xi_rot_next	= tmp_x_i;
							yi_rot_next	= tmp_y_i;							
						end						
					end
					endcase

					if($signed(y_i) < 0) begin
						dir_rot_next   = 1'b1;
					end else begin
						dir_rot_next   = 1'b0;
					end
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
					if (index < ITER_NUM) begin
						if(index == 4 || index == 13) begin
							if (flag == 0) begin
								index_next = index;	
								flag_next = 1;
							end else begin
								index_next = index + 1;	
								flag_next = 0;
							end
						end else begin
							if($signed (yi_rot_next) > 0) begin
								dir_rot_next	= 1'b1;
							end 
							else begin
								dir_rot_next	= 1'b0;
							end
							index_next = index + 1;							
						end
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
	end
	
    always@(*) begin: OUTPUT_PROC
		x_o_next     = x_o;
		y_o_next     = y_o;
		theta_o_next = theta_o;
        if (reset_i == 1'b1) begin
            x_o_next     = {IO_WIDTH{1'b0}};
			y_o_next     = {IO_WIDTH{1'b0}};
		end 
		else begin
			if(state == SCALE_X &&  mas_done == 1) begin
				x_o_next = product [IO_WIDTH +9 :10];
			end 
			else if(state == SCALE_Y &&  mas_done == 1) begin
					y_o_next = product [IO_WIDTH +9 :10];
			end
			else begin

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