`timescale 1ns / 1ps
module SevenSegFourDig_tb;

reg clk, rst;
wire [7:0] designOut;
wire [3:0] anodeOut;
reg [15:0] dataIn [0:63];
reg [15:0] in;
reg [7:0] refOut;
reg [3:0] refIn;

integer errorFlag, ii;

SevenSegFourDig DUT(.clk(clk), .rst(rst), .in(in), .sevenSeg(designOut), .anode(anodeOut));

initial begin
	clk = 1;
	forever
		#5 clk = ~clk;
end

initial begin
	errorFlag = #1 0;
	rst       = #1 0;
	in    	 = #1 0;
	repeat(5) @(posedge clk);
	rst       = #1 1;
	@(posedge clk);
	rst       = #1 0;
	for(ii = 1; ii<=50; ii = ii +1) begin
		in = #1 $random;
		repeat(5)@(designOut, anodeOut) begin
			@(negedge clk);
			if(designOut !== refOut) begin
				$display("Output ERROR at time %d, expected %b, received %b", $time, refOut, designOut);
				errorFlag = errorFlag +1;
			end
			else
				$display("Output CORRECT at time %d, expected %b, received %b", $time, refOut, designOut);
		end
	end

	if(errorFlag == 0)
		$display("No ERRORs Found!");
	else
		$display("%d ERRORs Found!", errorFlag);
	$finish;
end


always@(*) begin
	case(anodeOut)
		4'b0111:refIn = in[15:12];
		4'b1011:refIn = in[11:8];
		4'b1101:refIn = in[7:4];
		default:refIn = in[3:0];
	endcase

	case(refIn[3:0])
		0: refOut = 8'b00000010;  //0
		1: refOut = 8'b10011110;  //1
		2: refOut = 8'b00100100;  //2
		3: refOut = 8'b00001100;  //3
		4: refOut = 8'b10011000;  //4
		5: refOut = 8'b01001000;  //5
		6: refOut = 8'b01000000;  //6
		7: refOut = 8'b00011110;  //7
		8: refOut = 8'b00000000;  //8
		9: refOut = 8'b00001000;  //9
		10: refOut = 8'b00010000; //A
		11: refOut = 8'b11000000; //b
		12: refOut = 8'b01100010; //C
		13: refOut = 8'b10000100; //d
		14: refOut = 8'b01100000; //e
		15: refOut = 8'b01110000; //f
	endcase
end


endmodule
