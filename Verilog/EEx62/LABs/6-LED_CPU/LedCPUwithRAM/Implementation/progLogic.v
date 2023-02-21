 module progLogic(
		clk, 
		rst, 
		switch, 
		enter, 
		addrWr, 
		dataWr, 
		wrEn
); 

input clk, rst,enter;
input [7:0] switch;
output reg [15:0] dataWr;
output reg [7:0] addrWr;
output reg wrEn;

reg enter_r, wrEnNext;
reg [1:0] state, stateNext;
reg [15:0] dataWrNext;
reg [7:0] addrWrNext;

wire posEnter;
assign posEnter = ~enter_r &  enter;

always@(posedge clk)begin
	state   <= #1 stateNext;
	enter_r <= #1 enter;	
	dataWr  <= #1 dataWrNext;
	addrWr  <= #1 addrWrNext;
	wrEn    <= #1 wrEnNext;
end

always @(*) begin
	 stateNext = state;
	dataWrNext = dataWr;
	addrWrNext = addrWr;
		wrEnNext = 0;
	if (rst) begin
		stateNext = 0;
    addrWrNext = 255; //for starting 0 th location from ram.
		wrEnNext = 0;
		dataWrNext = 0;
	end else begin
		case(state)
			0: begin 
				if (posEnter) begin
					dataWrNext[15:8] = switch;	
					stateNext = 1;
				end
			end
			1: begin
				if (posEnter) begin
					dataWrNext[7:0] = switch;
					stateNext = 0;
					wrEnNext = 1;
					addrWrNext = addrWr + 1;
				end
			end			
		endcase
	end
end

endmodule
