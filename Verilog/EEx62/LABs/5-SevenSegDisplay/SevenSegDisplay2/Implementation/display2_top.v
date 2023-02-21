`timescale 1ns / 1ps
module display2_top(clk, rst, dataIn, sevenSeg, anode);

input clk, rst;
input [7:0] dataIn;
output [3:0] anode;
output [7:0] sevenSeg;

parameter SCWIDTH = 15;
parameter RCWIDTH = 25;

wire [4:0] seg0, seg1, seg2, seg3;
wire en;
wire [2:0] state;

timer #(RCWIDTH) timer0(.clk(clk), .rst(rst), .en(en));
stateMachine stateMachine0(.clk(clk), .rst(rst), .en(en), .state(state));
SevenSegFourDigwithEnable #(SCWIDTH) SevenSegFourDigwithEnable0(.clk(clk), .rst(rst), .in({seg3, seg2, seg1, seg0}), .sevenSeg(sevenSeg), .anode(anode));
rotateDigit rotateDigit0(.in0({3'b000,dataIn[1:0]}), 
								 .in1({3'b000,dataIn[3:2]}), 
								 .in2({3'b000,dataIn[5:4]}), 
								 .in3({3'b000,dataIn[7:6]}), 
								 .in4(5'b10000), 
								 .in5(5'b10000), 
								 .in6(5'b10000), 
								 .state(state), 
								 .seg0(seg0), 
								 .seg1(seg1), 
								 .seg2(seg2), 
								 .seg3(seg3));
endmodule
