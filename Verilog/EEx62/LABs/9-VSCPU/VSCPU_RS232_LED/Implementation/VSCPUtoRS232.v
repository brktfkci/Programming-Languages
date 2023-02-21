`timescale 1ns / 1ps
module VSCPUtoRS232(clk, rst, rx, tx, anode, reset, freezeFlag, debug, ledOut);

parameter SIZE = 10;

input  wire clk, rst;
input  wire rx;

output wire [3:0] anode;
output wire reset;
output wire freezeFlag;
//output wire [7:0] addrDebug;
output wire [5:0] debug;
output wire tx;
output wire [7:0] ledOut;


wire [SIZE-1:0] addra, addrb;
wire [31:0] dina, douta, dinb, doutb;
wire [SIZE-1:0] pCounter;
wire rxDone;
wire wea, web;

assign debug[5:1] = 15;
assign debug[0] = rxDone;
//assign addrDebug = addra[7:0];
assign anode = 14;
/////////////////////////////////SUBMODULES///////////////////////////////////////	

RS232_interface RS232_interface
	(.clk(clk), .rst(rst), .rx(rx), .tx(tx), .reset(reset), .freezeFlag(freezeFlag), .rxDn(rxDone), .web(web), .pCounter(pCounter), .addrb(addrb), .dinb(dinb), .doutb(doutb));	

blk_mem_gen RAM
	(.clka(clk), .wea(wea), .addra(addra), .dina(dina), .douta(douta), 
	 .clkb(clk), .web(web), .addrb(addrb), .dinb(dinb), .doutb(doutb),
	 .ledOut(ledOut));

MPU VerySimpleCPU
	(.clk(clk),  .rst(reset), .wrEn(wea), .addr_toRAM(addra), .data_toRAM(dina), .data_fromRAM(douta), .freezeFlag(freezeFlag), .pCounter(pCounter));

endmodule
