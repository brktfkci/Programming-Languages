`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    16:36:48 01/19/2010 
// Design Name: 
// Module Name:    ps2_rx 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module ps2_rx
(
	input wire clk, rst,
	input wire ps2d, ps2c, rx_en,
	output reg rx_done_tick,
	output wire [7:0] dout
) ;
// s y m b o l i c s t a t e d e c l a r a t i o n
	localparam [1:0]
	idle = 2 'b00,
	dps = 2'b01,
	load = 2'b10;
 // s i g n u l d e c l a r a t i o n
	reg [1 : 0] state_reg , state_next ;
	reg [7:0] filter_reg;
	wire [7:0] filter_next ;
	reg f_ps2c_reg ;
	wire f_ps2c_next ;
	reg [3:0] n_reg , n_next ;
	reg [10:0] b_reg, b_next;
	wire fall_edge ;
// f i l t e r and f a l l i n g - e d g e t i c k g e n e r a t i o n f o r ps2c
always @ ( posedge clk , posedge rst )
	if (rst)
	begin
		filter_reg <= 0 ;
		f_ps2c_reg <= 0;
	end
	else
	begin
		filter_reg <= filter_next ;
		f_ps2c_reg <= f_ps2c_next ;
	end

	assign filter_next = {ps2c, filter_reg [7:1]};
	assign f_ps2c_next = (filter_reg==8'b11111111) ? 1'b1 :
								(filter_reg==8'b00000000) ? 1'b0 :
								f_ps2c_reg;
	assign fall_edge = f_ps2c_reg & ~f_ps2c_next ;
// FSMD
// FSMD s t a t e & d a t a r e g i s t e r s
always @ ( posedge clk , posedge rst )
	if (rst)
	begin
		state_reg <= idle;
		n_reg <= 0;
		b_reg <= 0;
	end
	else
	begin
		state_reg <= state_next ;
		n_reg <= n_next;
		b_reg <= b_next;
	end
// FSMD n e x t - s t a t e l o g i c
always @*
	begin
		state_next = state_reg;
		rx_done_tick = 1'b0;
		n_next = n_reg;
		b_next = b_reg;
		case (state_reg)
			idle :
				if (fall_edge & rx_en)
				begin
	// s h i f t in s t a r t b i t
					b_next = {ps2d, b_reg [10:1]} ;
					n_next = 4'b1001;
					state_next = dps;
				end
			dps: // 8 d a t u + I p a r i t y + I s t o p
				if (fall_edge)
				begin
					b_next = { ps2d , b_reg [10:1]} ;
					if (n_reg==0)
						state_next = load ;
					else
						n_next = n_reg - 1 ;
				end
			load : // I e x t r a c l o c k t o c o m p l e t e the l a s t s h i f t
				begin
				state_next = idle ;
				rx_done_tick = 1'b1 ;
				end
		endcase
	end
// o u t p u t
	assign dout = b_reg [8:1] ; // data bits
endmodule
