`timescale 1ns / 1ps

module pixelGeneration(clk, rst, pixel_x, pixel_y, pixel_tick, mouse_x, mouse_y, mouse_btn, video_on, rgb
    );
input clk, rst;
input pixel_tick;
input [9:0] pixel_x, pixel_y, mouse_x, mouse_y;
input [2:0] mouse_btn;
input video_on;
output reg[2:0] rgb;

wire square_on;
reg [2:0] rgbNext;
reg [1:0] color, colorNext;
reg [2:0] mouse_btn_reg;

always@(*)begin
	rgbNext = rgb;
	if(pixel_tick)begin
		rgbNext = 3'b000;
		if(video_on)begin
			if(square_on)
				case(color)
					0: rgbNext = 3'b001;
					1: rgbNext = 3'b100;
					2: rgbNext = 3'b011;
					3: rgbNext = 3'b101;
				endcase
			else
				rgbNext = 3'b110;
		end
	end
end

always@(*)begin
	colorNext = color;
	if(rst)
		colorNext = 0;
	else if (mouse_btn_reg[0]==0 && mouse_btn[0]!=0)
		colorNext = 1;
	else if (mouse_btn_reg[1]==0 && mouse_btn[1]!=0)
		colorNext = 2;
	else if (mouse_btn_reg[2]==0 && mouse_btn[2]!=0)
		colorNext = 3;
end

always@(posedge clk)begin
		rgb <= rgbNext ;
		color <= colorNext;
		mouse_btn_reg <= mouse_btn;
end

assign square_on = ((pixel_x > mouse_x && pixel_x < mouse_x+40) && (pixel_y > mouse_y && pixel_y < mouse_y+40));

endmodule
