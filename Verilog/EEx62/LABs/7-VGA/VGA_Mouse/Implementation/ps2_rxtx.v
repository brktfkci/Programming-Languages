`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:34:17 01/20/2010 
// Design Name: 
// Module Name:    ps2_rxtx 
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
module ps2_rxtx
(
input wire clk , rst ,
input wire wr_ps2,
inout wire ps2d, ps2c,
input wire [7:0] din,
output wire rx_done_tick , tx_done_tick ,
output wire [7:0] dout
); 
// s i g n a l d e c l a r a t i o n
wire tx_idle ;
// body
// i n s t a n t i a t e ps2 r e c e i v e r
ps2_rx ps2_rx_unit
(.clk(clk), .rst(rst), .rx_en(tx_idle),
. ps2d (ps2d) , . ps2c (ps2c) ,
.rx_done_tick(rx_done_tick), .dout(dout));
// i n s t a n t i a t e ps2 t r a n s m i t t e r
ps2_tx ps2_tx_unit
(.clk(clk), .rst(rst), .wr_ps2(wr_ps2),
. din(din) , . ps2d(ps2d), .ps2c (ps2c),
.tx_idle(tx_idle), .tx_done_tick(tx_done_tick));
endmodule
