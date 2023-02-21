`timescale 1ns / 1ps
module led_tb;

reg        clk, rst;
wire [7:0] dataOut;
reg  [7:0] data [0:63];
reg  [7:0] datasize;

integer errorFlag, ii, jj;

initial $readmemb("patterns", data);

// Instantiate the Unit Under Test (UUT)
led led0(.clk(clk), .rst(rst), .dataOut(dataOut));

initial begin
	clk = 1;
	forever
		#5 clk = ~clk;
end

initial begin
	datasize  = data[0];
	errorFlag = 0;
	rst       = 0;
	repeat(5) @(posedge clk);
	rst       <= #1 1;
	@(posedge clk);
	rst       <= #1 0;
	for(ii = 0; ii<5; ii = ii +1) begin
		jj = 0;
		repeat(datasize) @(dataOut) begin
			@(negedge clk);
			jj = jj +1;
			if(dataOut !== data[jj]) begin
				$display("Output Error at time %d, expected %h, received %h", $time, data[jj], dataOut);
				errorFlag = errorFlag +1;
			end
		end
	end
	if(errorFlag == 0)
		$display("No Errors Found!");
	else
		$display("%d Errors Found!", errorFlag);
	$finish;
end
endmodule
