`timescale 1ns / 1ps
module display_top_tb;

parameter SCWIDTH = 1;


// Inputs
reg clk;
reg rst;
reg [7:0] dataIn;

// Outputs
wire [7:0] sevenSeg;
wire [3:0] anode;

// Instantiate the Unit Under Test (UUT)
display_top #(SCWIDTH) uut(.clk(clk), .rst(rst), .dataIn(dataIn), .sevenSeg(sevenSeg), .anode(anode));

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
	dataIn = 0;
	#3;
	#40 dataIn = 8'hFF;
	#40 dataIn = 8'h87;
	#40 dataIn = 8'hAB;
	#40 dataIn = 8'h32;
	#40 $finish;
end
      
endmodule

