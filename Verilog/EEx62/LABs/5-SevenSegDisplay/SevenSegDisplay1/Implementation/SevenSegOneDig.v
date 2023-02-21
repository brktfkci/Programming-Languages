`timescale 1ns / 1ps

module SevenSegOneDig(in, sevenSeg);

input [3:0] in;
output reg [7:0] sevenSeg;

always @(*) begin
	sevenSeg = 8'b00000010;
	case(in[3:0])
		 0: sevenSeg = 8'b00000010;  //0
		 1: sevenSeg = 8'b10011110;  //1
		 2: sevenSeg = 8'b00100100;  //2
		 3: sevenSeg = 8'b00001100;  //3
		 default:sevenSeg = 8'b00000010;
	endcase
end

endmodule
