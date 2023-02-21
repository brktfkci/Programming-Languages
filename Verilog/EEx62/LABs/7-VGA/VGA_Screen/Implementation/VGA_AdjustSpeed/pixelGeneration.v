`timescale 1ns / 1ps
module pixelGeneration(push, pixel_x, pixel_y, video_on, rgb, clk, rst);

input [1:0] push;
input [9:0] pixel_x, pixel_y;
input video_on, clk, rst;
output reg [2:0] rgb;

wire square_on;
wire [1:0] posEnter;
reg [1:0] enter_r;

reg [1:0] square, squareNext;
reg [9:0] myX, myY, myXNext, myYNext;
reg [25:0] freq, freqNext;
reg [25:0] count, countNext;

wire top_wall, bot_wall, right_wall, left_wall;  // top, bottom, right, left wall
assign top_wall   = (myY > 1019);
assign bot_wall   = (myY > 480 && myY < 1019);
assign right_wall = (myX > 607 && myX < 1019);
assign left_wall  = (myX > 1019);

always @(posedge clk) begin
		enter_r <= #1 push;
		myX     <= #1 myXNext;
		myY     <= #1 myYNext;
		freq    <= #1 freqNext;	
		square  <= #1 squareNext;
		count   <= #1 countNext;
end

always @(*) begin
	rgb = 3'b000;
	freqNext = freq;
	countNext = count;
	squareNext = square;
	myXNext = myX;
	myYNext = myY;
	if(rst) begin
		freqNext = 33_554_432;
		myXNext = 300;
		myYNext = 300;	
		squareNext = 0;
		countNext = 0;			
	end else begin
		countNext = count + 1;
		if(video_on) begin
			if(square_on)			
				rgb = 3'b100;
			else
				rgb = 3'b110;
		end
		if(count == freq)begin
			countNext = 0;
			case(square)
			0:begin
				if(bot_wall)begin
					squareNext = 3;
				end else if(right_wall)begin
					squareNext = 2;
				end else begin
					myXNext = myX + 10;
					myYNext = myY + 10;
				end
			end
			1:begin
				if(top_wall)begin
					squareNext = 2;
				end else if(left_wall)begin
					squareNext = 3;
				end else begin
					myXNext = myX - 10;
					myYNext = myY - 10;	
				end							
			end
			2:begin
				if(bot_wall)begin
					squareNext = 1;
				end else if(left_wall)begin
					squareNext = 0;
				end else begin
					myXNext = myX - 10;
					myYNext = myY + 10;	
				end									
			end
			3:begin
				if(top_wall)begin
					squareNext = 0;
				end else if(right_wall)begin
					squareNext = 1;
				end else begin
					myXNext = myX + 10;
					myYNext = myY - 10;	
				end						
			end
			endcase
		end
		case(posEnter)
			2'b01: freqNext = freq + 4_194_304;
			2'b10: freqNext = freq - 4_194_304;
			default: freqNext = freq;
		endcase
	end
end

assign posEnter = ~enter_r &  push;
assign square_on = ((pixel_x >  myX && pixel_x < myX + 40) && (pixel_y > myY && pixel_y < myY + 40));

endmodule
