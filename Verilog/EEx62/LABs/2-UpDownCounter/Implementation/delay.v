
module delay(rst, clk, out);
	input clk;
	input rst;
	
	output out;
	
	parameter CNT_WIDTH = 26; // 2^CNT_WIDTH cycles is the delay
	
	reg [CNT_WIDTH-1:0] timer, timerNext;

	always @(posedge clk) begin
		timer <= #1 timerNext;
	end

	always @(*) begin
		if(rst) begin
			timerNext = 0;
		end else begin
			timerNext = timer + 1;
		end
	end

	assign out = (timer==0);
	
endmodule
