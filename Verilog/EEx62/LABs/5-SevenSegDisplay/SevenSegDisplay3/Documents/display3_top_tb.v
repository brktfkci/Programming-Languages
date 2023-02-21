`timescale 1ns / 1ps
module display3_top_tb;

parameter SCWIDTH = 1;	// sevenSegFourDigwithEnable digit frequency parameter 
parameter RCWIDTH = 0; // rotatingDigits2 rotating frequency parameter
parameter CNTSIZE = 0; // debounce delay parameter

// Inputs
reg clk;
reg rst;
reg vld;
reg [3:0] dataIn;

// Outputs
wire [3:0] ledOut;
wire [3:0] anode;
wire [7:0] sevenSeg;

// Instantiate the Unit Under Test (UUT)
display3_top #(SCWIDTH, RCWIDTH, CNTSIZE) uut(.clk(clk), .rst(rst), .vld(vld), .dataIn(dataIn), .ledOut(ledOut), .anode(anode), .sevenSeg(sevenSeg));

initial begin
	clk = 0;
	forever
		#5 clk = ~clk;
end

initial begin
	rst = 1;
	#73 rst = 0;
end

initial begin
	vld = 0;
	dataIn = 0;
	#40 dataIn = 4'hF;
	#2  vld = 1;
	#10 vld = 0;
	#40 dataIn = 4'h7;
	#2  vld = 1;
	#10 vld = 0;
	#40 dataIn = 4'h5;
	#2  vld = 1;
	#10 vld = 0;
	#40 dataIn = 4'h6;
	#2  vld = 1;
	#10 vld = 0;
	#40 dataIn = 4'h8;
	#2  vld = 1;
	#10 vld = 0;
	#40 $finish;
end

endmodule

