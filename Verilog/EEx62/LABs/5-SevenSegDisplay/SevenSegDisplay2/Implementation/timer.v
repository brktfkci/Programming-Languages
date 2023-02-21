`timescale 1ns / 1ps

module timer(clk, rst, en
    );
input clk, rst;
output en;

parameter RCWIDTH = 25;

reg [RCWIDTH-1:0] cnt, cntNext;

always@(posedge clk)begin
	cnt <= cntNext;
end

always@(*)begin
	if(rst)
		cntNext = 0;
	else
		cntNext = cnt + 1;
end

assign en = (cnt==0);

endmodule
