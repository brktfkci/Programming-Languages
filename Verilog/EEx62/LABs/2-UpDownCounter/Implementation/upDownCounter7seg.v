`timescale 1ns / 1ps
module upDownCounter7seg(upDown, rst, clk, sevenSegment, anode);

	input upDown;
	input rst;
	input clk;
	output reg [7:0] sevenSegment;
	output reg [3:0] anode;
	
	reg [3:0] number = 4'b0000, numberNext = 4'b0000;
	wire delayOut;
	
	delay delay(.rst(rst),.clk(clk),.out(delayOut));
	
	always @(posedge clk) begin
		number <= #1 numberNext;
	end

	always @(*) begin
		numberNext = number;
		if(rst)
			numberNext = 0; 
		else if(delayOut)
			if(upDown)
				numberNext = (number == 9) ? 0 : number + 1;
			else 
				numberNext = (number == 0) ? 9 : number - 1;
	end
	
	always @(*) begin
		anode  = 4'b0111;
		sevenSegment = 8'b1111_1111;
		if(rst) begin 
			anode  = 4'b0111;
			sevenSegment = 8'b1111_1111;
		end else begin
			case(numberNext)
			0: sevenSegment = 8'b11000000; 
			1: sevenSegment = 8'b11111001;
			2: sevenSegment = 8'b10100100;
			3: sevenSegment = 8'b10110000;
			4: sevenSegment = 8'b10011001;
			5: sevenSegment = 8'b10010010;
			6: sevenSegment = 8'b10000010;
			7: sevenSegment = 8'b11111000;
			8: sevenSegment = 8'b10000000;
			9: sevenSegment = 8'b10010000;
			default: sevenSegment = 8'b11000000;
			endcase
		end
	end

endmodule
