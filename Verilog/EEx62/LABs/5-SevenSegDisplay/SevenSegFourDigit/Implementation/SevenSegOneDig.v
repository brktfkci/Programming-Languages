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
		4: sevenSeg = 8'b10011000;  //4
		5: sevenSeg = 8'b01001000;  //5
		6: sevenSeg = 8'b01000000;  //6
		7: sevenSeg = 8'b00011110;  //7
		8: sevenSeg = 8'b00000000;  //8
		9: sevenSeg = 8'b00001000;  //9
		10: sevenSeg = 8'b00010000; //A
		11: sevenSeg = 8'b11000000; //b
		12: sevenSeg = 8'b01100010; //C
		13: sevenSeg = 8'b10000100; //d
		14: sevenSeg = 8'b01100000; //e
		15: sevenSeg = 8'b01110000; //f
	endcase
end

endmodule
