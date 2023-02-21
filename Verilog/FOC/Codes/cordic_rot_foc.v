//=================================================================================================
// cordic_rot_foc module declaration
//=================================================================================================
module cordic_rot_foc
#(
// Generic List
    // Specifies width of all input and output ports
    parameter g_STD_IO_WIDTH = 18,

    // Specifies width of all input and output ports
    parameter g_CORDIC_ROTATIONS = 15
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
    input [g_STD_IO_WIDTH-1:0] x_i,

    // y co-ordinate input for cordic
    input [g_STD_IO_WIDTH-1:0]  y_i,

    // angle input for cordic
    input [g_STD_IO_WIDTH-1:0]  theta_i,

    // Control output specifies that the cordic has finished its operation
    output reg done_o,

    // X co-ordinate output of cordic after rotation
    output reg [g_STD_IO_WIDTH-1:0] x_o,

    // Y co-ordinate output of cordic after rotation
    output reg [g_STD_IO_WIDTH-1:0]  y_o
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

wire [g_STD_IO_WIDTH-1:0]CORDIC_ANGLES_2048[0:15];

assign CORDIC_ANGLES_2048[0]   = 18'h08000;
assign CORDIC_ANGLES_2048[1]   = 18'h04B90;
assign CORDIC_ANGLES_2048[2]   = 18'h027ED;
assign CORDIC_ANGLES_2048[3]   = 18'h01444;
assign CORDIC_ANGLES_2048[4]   = 18'h00A2C;
assign CORDIC_ANGLES_2048[5]   = 18'h00517;
assign CORDIC_ANGLES_2048[6]   = 18'h0028C;
assign CORDIC_ANGLES_2048[7]   = 18'h00146;
assign CORDIC_ANGLES_2048[8]   = 18'h000A3;
assign CORDIC_ANGLES_2048[9]   = 18'h00051;
assign CORDIC_ANGLES_2048[10]  = 18'h00029;
assign CORDIC_ANGLES_2048[11]  = 18'h00014;
assign CORDIC_ANGLES_2048[12]  = 18'h0000A;
assign CORDIC_ANGLES_2048[13]  = 18'h00005;
assign CORDIC_ANGLES_2048[14]  = 18'h00003;
assign CORDIC_ANGLES_2048[15]  = 18'h00001;

localparam [1:0] IDLE = 2'd0,
                ROT  = 2'd1;

reg start_i_reg;
reg done_o_next;
reg [g_STD_IO_WIDTH-1:0] x_o_next;
reg [g_STD_IO_WIDTH-1:0] y_o_next;				
reg [1:0] s_state;
reg [1:0] s_state_next;
reg s_dir_rot;
reg s_dir_rot_next;
reg [3:0] s_index;
reg [3:0] s_index_next;
reg [g_STD_IO_WIDTH-1:0] s_xi_rot;
reg [g_STD_IO_WIDTH-1:0] s_xi_rot_next;
reg [g_STD_IO_WIDTH-1:0] s_yi_rot;
reg [g_STD_IO_WIDTH-1:0] s_yi_rot_next;
reg [g_STD_IO_WIDTH-1:0] s_theta_c;
reg [g_STD_IO_WIDTH-1:0] s_theta_c_next;
reg [g_STD_IO_WIDTH-1:0] s_theta_i;
reg [g_STD_IO_WIDTH-1:0] s_theta_i_next;
reg [g_STD_IO_WIDTH-1:0] s_xout;
reg [g_STD_IO_WIDTH-1:0] s_xout_next;
reg [g_STD_IO_WIDTH-1:0] s_yout;
reg [g_STD_IO_WIDTH-1:0] s_yout_next;
wire [4:0] s_cordic_rot;
wire [4:0] s_cordic_rot_m1;
wire [4:0] s_cordic_rot_m1_next;
wire [g_STD_IO_WIDTH:0] s_sum1;
wire [g_STD_IO_WIDTH:0] s_sum2;
wire [g_STD_IO_WIDTH:0] s_diff1;
wire [g_STD_IO_WIDTH:0] s_diff2;
wire [g_STD_IO_WIDTH:0] s_sum_theta;
wire [g_STD_IO_WIDTH:0] s_diff_theta;

//=================================================================================================
// Asynchronous blocks
//=================================================================================================
assign s_sum1 = $signed(s_xi_rot) + $signed($signed(s_yi_rot) >>> (s_index-1));
assign s_sum2 = $signed(s_yi_rot) + $signed($signed(s_xi_rot) >>>  (s_index-1));
assign s_diff1 = $signed(s_yi_rot) - $signed($signed(s_xi_rot) >>> (s_index-1));
assign s_diff2 = $signed(s_xi_rot) - $signed($signed(s_yi_rot) >>> (s_index-1));
assign s_sum_theta = s_theta_c + CORDIC_ANGLES_2048[s_index];
assign s_diff_theta = s_theta_c - CORDIC_ANGLES_2048[s_index];
assign s_cordic_rot = g_CORDIC_ROTATIONS;
assign s_cordic_rot_m1 = g_CORDIC_ROTATIONS - 1'b1;

//------------------------------------------------------------------------
// Name       : MICRO_ROT_PROC
// Description: Process to compute micro rotations
//------------------------------------------------------------------------
	always@(posedge sys_clk_i) begin
		start_i_reg  	<= #1 start_i;
		s_xout  	<= #1 s_xout_next;
		s_yout  	<= #1 s_yout_next;
	end
	
	always@(*) begin:MICRO_ROT_PROC
		s_xout_next = s_xout;
		s_yout_next = s_yout;
		if(s_dir_rot == 1'b1) begin
			s_xout_next = s_sum1[g_STD_IO_WIDTH-1:0];
			s_yout_next = s_diff1[g_STD_IO_WIDTH-1:0];
		end 
		else begin
			s_xout_next = s_diff2[g_STD_IO_WIDTH-1:0];
			s_yout_next = s_sum2[g_STD_IO_WIDTH-1:0];
		end
	end
//------------------------------------------------------------------------
// Name       : CORDIC_FSM_PROC
// Description: FSM implements cordic operations
//------------------------------------------------------------------------
	always @(posedge sys_clk_i) begin
		s_state  	<= #1 s_state_next;
		s_xi_rot 	<= #1 s_xi_rot_next;
		s_yi_rot 	<= #1 s_yi_rot_next;
		s_index		<= #1 s_index_next;
		s_theta_c	<= #1 s_theta_c_next;
		s_theta_i	<= #1 s_theta_i_next;
		s_dir_rot 	<= #1 s_dir_rot_next;
	end
	
	always @(*) begin :CORDIC_FSM_PROC
		s_state_next   =  s_state;
		s_xi_rot_next  =  s_xi_rot;
		s_yi_rot_next  =  s_yi_rot;
		s_index_next   =  s_index;
		s_theta_c_next =  s_theta_c;
		s_theta_i_next =  s_theta_i;
		s_dir_rot_next =  s_dir_rot;
		if (reset_i == 1'b0) begin
			s_state_next	= IDLE;
			done_o	= 1'b0;
			s_xi_rot_next	= {g_STD_IO_WIDTH{1'b0}};
			s_yi_rot_next	= {g_STD_IO_WIDTH{1'b0}};
			s_index_next	= 0;
			s_theta_c_next	= {g_STD_IO_WIDTH{1'b0}};
			s_theta_i_next	= {g_STD_IO_WIDTH{1'b0}};
			s_dir_rot_next	= 1'b0;
		end 
		else begin
			case (s_state)
//------------------
// IDLE state
//------------------
			IDLE :
			begin
				done_o	= 1'b0;
				s_index_next	= 1;
				s_dir_rot_next	= 1'b0;
				if(start_i_reg == 1'b1) begin
					s_theta_c_next	= CORDIC_ANGLES_2048[0];
					s_xi_rot_next	= x_i;
					s_yi_rot_next	= y_i;
					s_theta_i_next	= theta_i;
					s_state_next	= ROT;
				end
			end
//------------------
// ROT state
//------------------
			ROT : 
			begin
				if(s_index == s_cordic_rot_m1[3:0]) begin
					s_state_next = IDLE;
					done_o  = 1'b1;
				end 
				else begin
					s_xi_rot_next = s_xout;
					s_yi_rot_next = s_yout;
					if (s_index < s_cordic_rot[3:0]) begin
						if($signed (s_theta_c) < $signed({2'd0,s_theta_i[g_STD_IO_WIDTH-3 : 0]})) begin
							s_dir_rot_next	= 1'b0;
							s_theta_c_next	= s_sum_theta[g_STD_IO_WIDTH-1:0];
						end 
						else begin
							s_dir_rot_next	= 1'b1;
							s_theta_c_next	= s_diff_theta[g_STD_IO_WIDTH-1:0];
						end
						s_index_next = s_index + 1;
					end
					s_state_next = ROT;
				end
			end
//------------------
// DEFAULT state
//------------------
			default: 
			begin
				s_xi_rot_next	= {g_STD_IO_WIDTH{1'b0}};
				s_yi_rot_next	= {g_STD_IO_WIDTH{1'b0}};
				s_state_next	= IDLE;
			end
			endcase
		end
	end

//------------------------------------------------------------------------
// Name       : OUTPUT_PROC
// Description: Assigns outputs of cordic
//------------------------------------------------------------------------
    always@(posedge sys_clk_i) begin
		x_o <= #1 x_o_next;
		y_o <= #1 y_o_next;
	end
	
    always@(*) begin: OUTPUT_PROC
		x_o_next = x_o;
		y_o_next = y_o;
        if (reset_i == 1'b0) begin
            x_o_next = {g_STD_IO_WIDTH{1'b0}};
			y_o_next = {g_STD_IO_WIDTH{1'b0}};
		end 
		else begin
            if ((s_state == ROT) && (s_index_next == s_cordic_rot_m1[3:0])) begin
                if(s_theta_i[g_STD_IO_WIDTH-1 : g_STD_IO_WIDTH-2] == 2'd0) begin
					x_o_next = s_xi_rot;
					y_o_next = s_yi_rot;
				end
				else if(s_theta_i[g_STD_IO_WIDTH-1 : g_STD_IO_WIDTH-2] == 2'd1) begin
					x_o_next = -s_yi_rot;
					y_o_next = s_xi_rot;
				end
				else if(s_theta_i[g_STD_IO_WIDTH-1 : g_STD_IO_WIDTH-2] == 2'd2) begin
					x_o_next = -s_xi_rot;
					y_o_next = -s_yi_rot;
				end
				else if(s_theta_i[g_STD_IO_WIDTH-1 : g_STD_IO_WIDTH-2] == 2'd3) begin
					x_o_next = s_yi_rot;
					y_o_next = -s_xi_rot;
				end
			end
        end
    end

endmodule