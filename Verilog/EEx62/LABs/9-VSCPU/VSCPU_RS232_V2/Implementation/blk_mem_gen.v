`timescale 1ns/1ps
module blk_mem_gen 
	(
	input clka,
	input wea,
	input [9 : 0] addra,
	input [31 : 0] dina,
	output reg [31 : 0] douta,
	
	input clkb,
	input web,
	input [9 : 0] addrb,
	input [31 : 0] dinb,
	output reg [31 : 0] doutb
	);
	
parameter DEPTH = 512;

//synthesis attribute ram_style of memory is block
reg [31:0] memory[0:DEPTH-1]; // pragma attribute memory ram_block TRUE

always @(posedge clka) begin
	douta <= #1 memory[addra];
	if (wea) 
		memory[addra] <= #1 dina;
end 

always @(posedge clkb) begin
	doutb <= #1 memory[addrb];
	if (web)
		memory[addrb] <= #1 dinb;
end
	
endmodule
