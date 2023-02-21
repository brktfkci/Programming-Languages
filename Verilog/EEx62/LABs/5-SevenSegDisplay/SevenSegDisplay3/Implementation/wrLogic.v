`timescale 1ns / 1ps

module wrLogic(clk, rst, enter, sw, seg0wr, seg1wr, seg2wr, seg3wr
    );
input clk, rst, enter;
input [3:0] sw;
output reg [4:0] seg0wr, seg1wr, seg2wr, seg3wr;

reg [1:0] state, stateNext;
reg [4:0] seg0wrNext, seg1wrNext, seg2wrNext, seg3wrNext;
reg [4:0] tmpSeg0wrNext, tmpSeg1wrNext, tmpSeg2wrNext, tmpSeg3wrNext;
reg [4:0] tmpSeg0wr, tmpSeg1wr, tmpSeg2wr, tmpSeg3wr;
//for clean input
reg enter_r;
wire posEnter;
assign posEnter = ~enter_r &  enter;
////
always@(posedge clk)begin
	state     <= #1 stateNext;
	enter_r   <= #1 enter;
	seg0wr    <= #1 seg0wrNext;
	seg1wr    <= #1 seg1wrNext;
	seg2wr    <= #1 seg2wrNext;
	seg3wr    <= #1 seg3wrNext;
	tmpSeg0wr <= #1 tmpSeg0wrNext;
	tmpSeg1wr <= #1 tmpSeg1wrNext;
	tmpSeg2wr <= #1 tmpSeg2wrNext;
	tmpSeg3wr <= #1 tmpSeg3wrNext;	
end

always@(*)begin
	stateNext  = state;
	seg0wrNext = seg0wr;
	seg1wrNext = seg1wr;
	seg2wrNext = seg2wr;
	seg3wrNext = seg3wr;
	tmpSeg0wrNext = tmpSeg0wr;
	tmpSeg1wrNext = tmpSeg1wr;
	tmpSeg2wrNext = tmpSeg2wr;
	tmpSeg3wrNext = tmpSeg3wr;
	if (rst) begin
		stateNext  = 0;
		seg0wrNext = 5'b11111;
		seg1wrNext = 5'b11111;
		seg2wrNext = 5'b11111;
		seg3wrNext = 5'b11111;
		tmpSeg0wrNext = 5'b11111;
		tmpSeg1wrNext = 5'b11111;
		tmpSeg2wrNext = 5'b11111;
		tmpSeg3wrNext = 5'b11111;
	end
	else if(posEnter) begin
		stateNext = state + 1;
		case(state)
			0:begin
				tmpSeg0wrNext = {0, sw};
				tmpSeg1wrNext = 5'b11111;
				tmpSeg2wrNext = 5'b11111;
				tmpSeg3wrNext = 5'b11111;
			end
			1:begin
				tmpSeg0wrNext = tmpSeg0wr;
				tmpSeg1wrNext = {0, sw};
				tmpSeg2wrNext = 5'b11111;
				tmpSeg3wrNext = 5'b11111;
			end
			2:begin
				tmpSeg0wrNext = tmpSeg0wr;
				tmpSeg1wrNext = tmpSeg1wr;
				tmpSeg2wrNext = {0, sw};
				tmpSeg3wrNext = 5'b11111;
			end
			3:begin
				tmpSeg0wrNext = tmpSeg0wr;
				tmpSeg1wrNext = tmpSeg1wr;
				tmpSeg2wrNext = tmpSeg2wr;
				tmpSeg3wrNext = {0, sw};		
			end
		endcase			
	end
	else if(stateNext == 0) begin
		seg0wrNext = tmpSeg0wr;
		seg1wrNext = tmpSeg1wr;
		seg2wrNext = tmpSeg2wr;
		seg3wrNext = tmpSeg3wr;
	end
end

endmodule
