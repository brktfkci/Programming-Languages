`timescale 1ns / 1ps

module SevenSegFourDig(in, clk, rst, sevenSeg, anode);

input clk, rst;
input [15:0] in;
output [7:0] sevenSeg;
output reg [3:0] anode;

parameter SCWIDTH = 15;

reg [SCWIDTH:0] cnt, cntNext;
reg [3:0] inOneDig;

SevenSegOneDig SevenSegOneDig(.in(inOneDig), .sevenSeg(sevenSeg));

always@(posedge clk) begin
	cnt <= cntNext;
end

always@(*) begin
	if(rst) begin
		cntNext = 0;
		anode = 4'b1111;
		inOneDig = 4'b1111;
	end else begin
		cntNext = cnt + 1;
		case(cnt[SCWIDTH:SCWIDTH-1])
			2'b00:begin
				inOneDig = in[3:0];
				anode = 4'b0111;
			end
			2'b01:begin
				inOneDig = in[7:4];
				anode = 4'b1011;
			end
			2'b10:begin
				inOneDig = in[11:8];
				anode = 4'b1101;
			end
			2'b11:begin
				inOneDig = in[15:12];
				anode = 4'b1110;
			end
		endcase
	end
end
endmodule
