`timescale 1ns / 1ps
module pixelGeneration(push, pixel_x, pixel_y, video_on, rgb,clk,rst);

input [3:0] push;
input [9:0] pixel_x, pixel_y;
input video_on,clk,rst;
output reg [2:0] rgb;

wire square_on;
wire [3:0] posEnter;

reg [9:0] myX,myY,myXNext,myYNext;
reg [3:0] enter_r;


assign square_on = ((pixel_x >  myX && pixel_x < myX+40) && (pixel_y > myY && pixel_y < myY+40));
assign posEnter = ~enter_r &  push;

always @(posedge clk) begin
		myX <= #1 myXNext;
		myY <= #1 myYNext;
enter_r <= #1 push;
end

always @(*) begin
	myXNext = myX;
	myYNext = myY;
	rgb = 3'b110;
	if (rst) begin
		myXNext = 320;
		myYNext = 220;
	end 
	else if(video_on) begin
		if(square_on)
			rgb = 3'b101;
		else
			rgb = 3'b110;
	end
	case(posEnter)
		4'b0001: myXNext = myX + 20;
		4'b0010: myXNext = myX - 20;
		4'b0100: myYNext = myY + 20;
		4'b1000: myYNext = myY - 20;
		default : begin
		myXNext = myX;
		myYNext = myY;
		end
	endcase		
end

endmodule
