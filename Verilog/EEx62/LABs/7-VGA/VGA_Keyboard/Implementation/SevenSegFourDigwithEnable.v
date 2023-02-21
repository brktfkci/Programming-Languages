`timescale 1ns / 1ps

module SevenSegFourDigwithEnable(clk, rst, in, sevenSeg, anode);
input clk, rst;
input [19:0] in;
output [7:0] sevenSeg;
output reg [3:0] anode;

parameter SCWIDTH = 15;

reg [SCWIDTH:0] cnt, cntNext;
reg [4:0] inOneDig;

SevenSegOneDigwithEnable inst0(.in(inOneDig), .sevenSeg(sevenSeg));

always@(posedge clk) begin
	cnt <= cntNext;
end

always@(*) begin
	if(rst) begin
		cntNext = 0;
		anode = 4'b1111;
		inOneDig = 5'b11111;
	end else begin
		cntNext = cnt + 1;
		case(cnt[SCWIDTH:SCWIDTH-1])
			2'b00:begin
				inOneDig = in[19:15];
				anode = 4'b0111;
			end
			2'b01:begin
				inOneDig = in[14:10];
				anode = 4'b1011;
			end
			2'b10:begin
				inOneDig = in[9:5];
				anode = 4'b1101;
			end
			2'b11:begin
				inOneDig = in[4:0];
				anode = 4'b1110;
			end
		endcase
	end
end
endmodule


module SevenSegOneDigwithEnable(in, sevenSeg);
input [4:0] in;
output reg [7:0] sevenSeg;

always@(*) begin
	if(in[4])
		sevenSeg = 8'b11111111;        //Empty
	else begin
		case(in[3:0])
			0:  sevenSeg = 8'b00000011;  //0
			1:  sevenSeg = 8'b10011111;  //1
			2:  sevenSeg = 8'b00100101;  //2
			3:  sevenSeg = 8'b00001101;  //3
			4:  sevenSeg = 8'b10011001;  //4
			5:  sevenSeg = 8'b01001001;  //5
			6:  sevenSeg = 8'b01000001;  //6
			7:  sevenSeg = 8'b00011111;  //7
			8:  sevenSeg = 8'b00000001;  //8
			9:  sevenSeg = 8'b00001001;  //9
			10: sevenSeg = 8'b00010001;  //A
			11: sevenSeg = 8'b11000001;  //b
			12: sevenSeg = 8'b01100011;  //C
			13: sevenSeg = 8'b10000101;  //D
			14: sevenSeg = 8'b01100001;  //E
			15: sevenSeg = 8'b01110001;  //F
		endcase
	end
end
endmodule
