`timescale 1ns / 1ps
module RS232_interface(clk, rst, rx, tx, reset, freezeFlag, pCounter, addrb, rxDn, dinb, doutb, web);		
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
	
input clk, rst;
input rx;
input [SIZE-1:0] pCounter;
input [31:0] doutb;


output tx;
output web;
output rxDn;
output reset;
output freezeFlag;
output [SIZE-1:0] addrb;
output [31:0] dinb;

wire [7:0] rxOut, txIn;
wire rxDone;

assign rxDn = rxDone;
/////////////////////////////////SUBMODULES///////////////////////////////////////			
		
mod_m_counter #(.M(DVSR), .N(DVSR_BIT)) baud_gen_unit  
(.clk(clk), .reset(rst), .q(), .max_tick(tick));

uart_rx #(.DBIT(DBIT), .SB_TICK(SB_TICK)) uart_rx_unit 
(.clk(clk), .reset(rst), .rx(rx), .s_tick(tick), .rx_done_tick(rxDone), .dout(rxOut));

uart_tx #(.DBIT(DBIT), .SB_TICK(SB_TICK)) uart_tx_unit
(.clk(clk), .reset(rst), .tx_start(txStart), .s_tick(tick), .din(txIn), .tx_done_tick(txDone), .tx(tx));
    
RStoRAM RS
(.clk(clk), .rst(rst), .rxDone(rxDone), .rxIn(rxOut), .txStart(txStart), .txDone(txDone), 
 .txOut(txIn), .en(web), .addr(addrb), .dataOut(dinb), .dataIn(doutb), .pCounter(pCounter),
 .resetMPU(reset), .freezeFlag(freezeFlag));


endmodule
