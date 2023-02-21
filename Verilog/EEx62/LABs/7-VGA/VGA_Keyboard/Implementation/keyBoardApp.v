module keyBoardApp(clk, rst, ps2d, ps2c, sevenSeg, anode);
input clk, rst;
input ps2d, ps2c;
output [7:0] sevenSeg;
output [3:0] anode;

wire rx_done_tick;
wire [7:0] rx_data;
wire [19:0] sevenIn;
 

ps2_rx ps2_rx(.clk(clk), .rst(rst), .ps2d(ps2d), .ps2c(ps2c), .rx_en(1'b1), .rx_done_tick(rx_done_tick), .dout(rx_data));

SevenSegFourDigwithEnable SevenSegFourDigwithEnable(.clk(clk), .rst(rst), .in(sevenIn), .sevenSeg(sevenSeg), .anode(anode));

keyBoardCore keyBoardCore(.clk(clk), .rst(rst), .rx_done_tick(rx_done_tick), .rx_data(rx_data), .sevenOut(sevenIn));

endmodule
