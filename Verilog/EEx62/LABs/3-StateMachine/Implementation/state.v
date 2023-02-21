`timescale 1ns / 1ps
module state_machine(btn, clk, ledout, rst);
input btn, clk, rst;
output reg [2:0] ledout;

reg [2:0] ledoutNext;
reg [1:0] count,countNext;
wire btnD;

debounce debounce(.in(btn), .clk(clk), .out(btnD), .rst(rst));

always@(posedge clk) begin
	ledout  <= #1 ledoutNext;
	count   <= #1 countNext;
end
always@(*)begin
	ledoutNext = ledout;
	countNext  = count;
	if(rst)begin
		ledoutNext = 3'b001;
		countNext  = 0;
	end				
	else if(btnD)begin
		countNext  = count + 1;
	end
	case(count)		
		2'b00:ledoutNext = 3'b001;
		2'b01:ledoutNext = 3'b010;
		2'b10:ledoutNext = 3'b011;
		2'b11:ledoutNext = 3'b100;
	endcase	
end
		
endmodule
