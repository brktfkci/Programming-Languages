`timescale 1ns / 1ps

module VGAmouse 
(
	input wire clk, rst,
	inout wire ps2d, ps2c,
	output wire hsync, vsync,
	output wire [2:0] rgb
);

wire [9 :0] pixel_x, pixel_y;
wire video_on , pixel_tick;

wire [9:0] mouse_x, mouse_y;
wire [2:0] mouse_btn;

vgaSync vgaSync
	( .clk(clk), .rst(rst), .hsync(hsync), .vsync(vsync),
	.video_on(video_on), .p_tick(pixel_tick),
	.pixel_x(pixel_x), .pixel_y(pixel_y));
	
pixelGeneration pixelGeneration
	(.clk(clk), .rst(rst), .pixel_x(pixel_x), .pixel_y(pixel_y), .pixel_tick(pixel_tick), 
	.mouse_x(mouse_x), .mouse_y(mouse_y), .mouse_btn(mouse_btn), 
	.video_on(video_on), .rgb(rgb));

mouse mouse_unit
	(.clk(clk), .rst(rst), .ps2d(ps2d), .ps2c(ps2c),
	.mouse_x(mouse_x), .mouse_y(mouse_y), .btnm(mouse_btn)) ;
	
endmodule
