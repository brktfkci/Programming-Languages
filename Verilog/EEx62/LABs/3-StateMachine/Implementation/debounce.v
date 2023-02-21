module debounce(in,clk,out,rst);
input in,clk,rst;
output reg out;

parameter CNTSIZE = 10;

reg infirstNext, in_r, infirst;
reg state, stateNext;
reg [CNTSIZE:0] cnt, cntNext;
wire triger;

always@(posedge clk) begin
	in_r    <= #1 in;
	cnt     <= #1 cntNext;
	state   <= #1 stateNext;
	infirst <= #1 infirstNext;
end

always@(*)begin
	cntNext = cnt;
	stateNext = state;
	infirstNext = infirst;
	if(rst)begin
		stateNext = 0;
		cntNext = 0;
		infirstNext = 0;
	end else begin
		case(state)
			0: begin
					if(triger)begin
						stateNext = 1;
						infirstNext = in;
						cntNext = cnt + 1;
					end
				end
			1: begin
				if(cnt == 0)
					stateNext = 0;
				else
					cntNext = cnt + 1;
			end
		endcase
	end
end

always@(*)begin
	if(cnt!=0 || state == 1)
		out = infirst;
	else
		out = in;
end

assign triger = in_r^in;

endmodule
