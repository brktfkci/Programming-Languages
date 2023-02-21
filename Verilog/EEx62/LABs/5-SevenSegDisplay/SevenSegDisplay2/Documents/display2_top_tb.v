`timescale 1ns / 1ps
module display2_top_tb;

parameter SCWIDTH = 1;
parameter RCWIDTH = 1;

// Inputs
reg clk;
reg rst;
reg [7:0] dataIn;

// Outputs
wire [7:0] sevenSeg;
wire [3:0] anode;

// Instantiate the Unit Under Test (UUT)
display2_top #(SCWIDTH, RCWIDTH) uut(.clk(clk), .rst(rst), .dataIn(dataIn), .sevenSeg(sevenSeg), .anode(anode));

// Create clock input
initial begin
	clk = 0;
	forever
		#5 clk = ~clk;
end

// Create reset input
initial begin
	rst = 0;
	#102  rst = 1;
	#2   rst = 0;
	#100  rst = 1;
	#2   rst = 0;
	#100  rst = 1;
	#2   rst = 0;
	#100 rst = 1;
	#2   rst = 0;
	#100;
	$finish;
end

// Create dataIn inputs
initial begin
	dataIn = 0;
	# 61  dataIn = 8'hFF;
	# 121  dataIn = 8'hF0;
	# 181  dataIn = 8'h0F;
	# 241 dataIn = 8'hAB;
end
      
endmodule
