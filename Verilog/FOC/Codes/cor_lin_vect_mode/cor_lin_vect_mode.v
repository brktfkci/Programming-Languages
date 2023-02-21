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
    input [IO_WIDTH-1:0] x_i,

    // y co-ordinate input for cordic
    input [IO_WIDTH-1:0]  y_i,

    // angle input for cordic
    input [IO_WIDTH-1:0]  theta_i,

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
wire [IO_WIDTH-1:0]ITER_ANG[0:15];

assign ITER_ANG[0]   = 18'h08000;
assign ITER_ANG[1]   = 18'h04000;
assign ITER_ANG[2]   = 18'h02000;
assign ITER_ANG[3]   = 18'h01000;
assign ITER_ANG[4]   = 18'h00800;
assign ITER_ANG[5]   = 18'h00400;
assign ITER_ANG[6]   = 18'h00200;
assign ITER_ANG[7]   = 18'h00100;
assign ITER_ANG[8]   = 18'h00080;
assign ITER_ANG[9]   = 18'h00040;
assign ITER_ANG[10]  = 18'h00020;
assign ITER_ANG[11]  = 18'h00010;
assign ITER_ANG[12]  = 18'h00008;
assign ITER_ANG[13]  = 18'h00004;
assign ITER_ANG[14]  = 18'h00002;
assign ITER_ANG[15]  = 18'h00001;

localparam [1:0] IDLE = 2'd0, ROT  = 2'd1;

reg [IO_WIDTH-1:0] x_o_next;
reg [IO_WIDTH-1:0] y_o_next;				
reg [IO_WIDTH-1:0] theta_o_next;	
reg done_o_next;			
reg [1:0] state, state_next;
reg flag, flag_next;
reg [3:0] index, index_next;
reg dir_rot, dir_rot_next;
reg [IO_WIDTH-1:0] xi_rot, xi_rot_next;
reg [IO_WIDTH-1:0] yi_rot, yi_rot_next;
reg [IO_WIDTH-1:0] theta_rot, theta_rot_next;

reg [IO_WIDTH-1:0] cons_xi;
reg [IO_WIDTH-1:0] tmp_xi;
reg [IO_WIDTH-1:0] tmp_yi;

reg [IO_WIDTH-1:0] x_out;
reg [IO_WIDTH-1:0] y_out;
reg [IO_WIDTH-1:0] theta_out;

wire [IO_WIDTH:0] dir_1_dif_xi_rot;
wire [IO_WIDTH:0] dir_2_sum_xi_rot;
wire [IO_WIDTH:0] dir_1_sum_yi_rot;
wire [IO_WIDTH:0] dir_2_dif_yi_rot;
wire [IO_WIDTH:0] dir_1_dif_theta_rot;
wire [IO_WIDTH:0] dir_2_sum_theta_rot;


//=================================================================================================
// Asynchronous blocks
//=================================================================================================
assign dir_1_dif_xi_rot = cons_xi;
assign dir_1_sum_yi_rot = $signed(yi_rot_next) + $signed($signed(cons_xi) >>> (index_next-1));
assign dir_1_dif_theta_rot = ~reset_i ? theta_rot_next - ITER_ANG[index_next-1] : 0;

assign dir_2_sum_xi_rot = cons_xi;
assign dir_2_dif_yi_rot = $signed(yi_rot_next) - $signed($signed(cons_xi) >>> (index_next-1));
assign dir_2_sum_theta_rot = ~reset_i ? theta_rot_next + ITER_ANG[index_next-1] : 0;

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
		flag       <= #1 flag_next;
	end
	
	always @(*) begin :CORDIC_FSM_PROC
		xi_rot_next    = xi_rot;	 
		yi_rot_next    = yi_rot;
		theta_rot_next = theta_rot;
		dir_rot_next   = dir_rot;      
		state_next     = state; 	          
		index_next     = index;
		flag_next      = flag;
        done_o_next    = 1'b0;
		if (reset_i == 1'b1) begin
			xi_rot_next	   = {IO_WIDTH{1'b0}};
			yi_rot_next	   = {IO_WIDTH{1'b0}}; 
            theta_rot_next = {IO_WIDTH{1'b0}};  
            dir_rot_next   = 1'b0;     
            flag_next      = 1'b0;     
			state_next	   = IDLE;
            index_next	   = 1;
			done_o_next    = 1'b0;			
		end 
		else begin
			case (state)
//------------------
// IDLE state
//------------------
			IDLE :
			begin
                case({y_i[IO_WIDTH-1], x_i[IO_WIDTH-1]})
                0:begin
                    if (x_i < yi) begin
                        cons_xi = y_i;
                        yi_rot_next = x_i;                              
                    end else begin
                        cons_xi = x_i;
                        yi_rot_next = y_i;                             
                    end
                end
                1:begin
                    tmp_xi = ~x_i + 1;                     
                    if(tmp_xi < y_i) begin
                        cons_xi = y_i;
                        yi_rot_next = tmp_xi;  
                    end else begin
                        cons_xi = tmp_xi;
                        yi_rot_next = y_i;                             
                    end                  
                end
                2:begin
                    tmp_yi = ~y_i + 1;
                    if(x_i < tmp_yi) begin
                        cons_xi = tmp_yi;
                        yi_rot_next = x_i;  
                    end else begin
                        cons_xi = tmp_xi;
                        yi_rot_next = y_i;                             
                    end                    
                end
                3:begin
                    if(x_i[IO_WIDTH-2:0] < y_i[IO_WIDTH-2:0]) begin
                        cons_xi = y_i;
                        yi_rot_next = x_i;  
                    end else begin
                        cons_xi = x_i;
                        yi_rot_next = y_i;                             
                    end                  
                end
                endcase            
				if(start_i == 1'b1) begin                
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
					done_o_next = 1'b1;	
					state_next  = IDLE;
					dir_rot_next   = 1'b0; 
					index_next	   = 1;	
				end 
				else begin
					xi_rot_next = x_out;
					yi_rot_next = y_out;
                    theta_rot_next = (index == 1) ? ITER_ANG[0] : theta_out;
					if (index < ITER_NUM) begin
						if($signed (yi_rot_next) < 0) begin
							dir_rot_next	= 1'b1;
						end 
						else begin
							dir_rot_next	= 1'b0;
						end
						index_next = index + 1;
					end
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
			if ((state == ROT) && (index == ITER_NUM))begin
				x_o_next = xi_rot;
                y_o_next = yi_rot;
			    theta_o_next = theta_rot;
			end else begin
                x_o_next     = {IO_WIDTH{1'b0}};
                y_o_next     = {IO_WIDTH{1'b0}};
                theta_o_next = {IO_WIDTH{1'b0}};            
            end
        end
    end

endmodule