`timescale 1ns / 1ps
module rs232(clk, rst, rx, tx, anode, reset, freezeFlag, addrDebug, debug);
// Default setting:
// 115,200 baud, 8 data bits, 1 stop bit, 2^2 FIFO
parameter DBIT = 8;       // # data bits
parameter SB_TICK = 16;   // # ticks for stop bits, 16/24/32
                          // for 1/1.5/2 stop bits
parameter DVSR = 27;      // baud rate divisor
                          // DVSR = 50M/(16*baud rate)
parameter DVSR_BIT = 8;   // # bits of DVSR
parameter FIFO_W = 2;     // # addr bits of FIFO
                          // # words in FIFO=2^FIFO_W


parameter SIZE = 10;

input  wire clk, rst;
input  wire rx;
output wire tx;

output wire [3:0] anode;
output wire reset;
output wire freezeFlag;
output wire [7:0] addrDebug;
output wire [5:0] debug;

wire [7:0] rxOut, txIn;
wire [SIZE-1:0] addra, addrb;
wire [31:0] dina, douta, dinb, doutb;
wire [SIZE-1:0] pCounter;
wire rxDone;

assign debug[5:1] = 15;
assign debug[0] = rxDone;
assign addrDebug = addra[7:0];
assign anode = 14;
/////////////////////////////////SUBMODULES///////////////////////////////////////	
mod_m_counter #(.M(DVSR), .N(DVSR_BIT)) baud_gen_unit  

	(.clk(clk), .reset(rst), .q(), .max_tick(tick));

uart_rx #(.DBIT(DBIT), .SB_TICK(SB_TICK)) uart_rx_unit 

	(.clk(clk), .reset(rst), .rx(rx), .s_tick(tick), .rx_done_tick(rxDone), .dout(rxOut));

uart_tx #(.DBIT(DBIT), .SB_TICK(SB_TICK)) uart_tx_unit

	(.clk(clk), .reset(rst), .tx_start(txStart), .s_tick(tick), .din(txIn), .tx_done_tick(txDone), .tx(tx));

blk_mem_gen RAM

	(.clka(clk), .wea(wea), .addra(addra), .dina(dina), .douta(douta), 
	.clkb(clk), .web(web), .addrb(addrb), .dinb(dinb), .doutb(doutb));

MPU #(SIZE) MPU

	(.clk(clk), .rst(reset), .wrEn(wea), .addr_toRAM(addra), .data_toRAM(dina), .data_fromRAM(douta), .freezeFlag(freezeFlag), .pCounter(pCounter));
RStoRAM RS

	(.clk(clk), .rst(rst), .rxDone(rxDone), .rxIn(rxOut), .txStart(txStart), .txDone(txDone), .txOut(txIn), .en(web), .addr(addrb), .dataOut(dinb), .dataIn(doutb), .pCounter(pCounter), .resetMPU(reset), .freezeFlag(freezeFlag));

endmodule
