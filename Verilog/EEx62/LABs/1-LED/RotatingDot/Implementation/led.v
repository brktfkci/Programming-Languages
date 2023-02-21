`timescale 1ns / 1ps
module led(clk, rst, dataOut);

input clk, rst;
output reg [7:0] dataOut;

reg [21:0] counter, counterNext;
reg [ 7:0] dataOutNext;

parameter COUNT = 22'h3FFFFF;

// registers
always @(posedge clk) begin
	counter <= #1 counterNext;
	dataOut <= #1 dataOutNext;
end

always @(*) begin
	if(rst) begin
		dataOutNext = 8'b1000_0000;
		counterNext = 0;
	end
	else if(counter == COUNT -1) begin 
		dataOutNext = {dataOut[0], dataOut[7:1]}; // leds shifting left to right  
		counterNext = 0;
	end
	else begin
		dataOutNext = dataOut;
		counterNext = counter +1;
	end
end

endmodule
