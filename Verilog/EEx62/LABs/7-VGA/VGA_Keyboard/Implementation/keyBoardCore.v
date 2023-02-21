module keyBoardCore(clk, rst, rx_done_tick, rx_data, sevenOut 
    );
input clk, rst;
input rx_done_tick;
input [7:0] rx_data;
output reg [19:0] sevenOut;

wire [4:0] binary_data;

key2Binary key2Binary(.key_code(rx_data), .binary(binary_data));

reg state, stateNext;
reg [1:0] flag, flagNext;
reg [4:0] seg0wr, seg1wr, seg2wr, seg3wr;
reg [4:0] seg0wrNext, seg1wrNext, seg2wrNext, seg3wrNext;

always @(posedge clk) begin
	state  <= #1 stateNext;
	flag   <= #1 flagNext;
	seg0wr <= #1 seg0wrNext;
	seg1wr <= #1 seg1wrNext;
	seg2wr <= #1 seg2wrNext;
	seg3wr <= #1 seg3wrNext;	
end

always@(*)begin
	stateNext  = state;
	flagNext   = flag;
	seg0wrNext = seg0wr;
	seg1wrNext = seg1wr;
	seg2wrNext = seg2wr;
	seg3wrNext = seg3wr;
	sevenOut = {seg3wr,seg2wr,seg1wr,seg0wr};
	if(rst)begin
		stateNext = 0;
		flagNext  = 0;
		seg0wrNext = 5'b11111;
		seg1wrNext = 5'b11111;
		seg2wrNext = 5'b11111;
		seg3wrNext = 5'b11111;		
	end else begin	
		case(state)
		0: begin
			if(rx_done_tick)
				if(rx_data == 8'hF0)
					stateNext = 1;
		end
		1: begin
			if(rx_done_tick)begin
				stateNext = 0;
				flagNext  = flag + 1;
				case(flag)
				0: seg0wrNext = binary_data;
				1: seg1wrNext = binary_data;
				2: seg2wrNext = binary_data;
				3: seg3wrNext = binary_data;					
				endcase
			end
		end
		endcase	
	end
end

endmodule
