`timescale 1ns / 1ps

module tb_cordic;

	// Inputs
	reg reset_i;
	reg sys_clk_i;
	reg start_i;
	reg [17:0] x_i;
	reg [17:0] y_i;
	reg [17:0] theta_i;

	// Outputs
	wire done_o;
	wire [17:0] x_o;
	wire [17:0] y_o;
	wire [17:0] theta_o;

	// Instantiate the Unit Under Test (UUT)
	cordic uut (
		.reset_i(reset_i), 
		.sys_clk_i(sys_clk_i), 
		.start_i(start_i), 
		.x_i(x_i), 
		.y_i(y_i), 
		.theta_i(theta_i), 
		.done_o(done_o), 
		.x_o(x_o), 
		.y_o(y_o), 
		.theta_o(theta_o)
	);

	integer ii;
	reg [19:0] data [0:13];

	initial $readmemh("data", data);

	initial begin
		sys_clk_i = 0;
		forever begin
			sys_clk_i = #5 ~sys_clk_i;
		end
	end

	initial begin
		// Initialize Inputs
		reset_i = 1;
		start_i = 0;
		x_i = 0;
		y_i = 0;
		ii = 0;
		theta_i = 0;		
		#50;
		reset_i = 0;	
		start_i = 1;
		x_i = 1024;
		y_i = 0;
		theta_i = 43690;
		repeat(14)@(posedge done_o) begin
			x_i <= #1 data[ii];
			ii = ii + 1;
			start_i = 1;
		end	
		$finish;
        
	end
      
endmodule

