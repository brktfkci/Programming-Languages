`timescale 1ns / 1ps
module UDC_tb;

reg clk, rst, upDown;
wire [7:0] dataOut;
wire [3:0] anodeOut;
reg [7:0] data [0:63];
reg [7:0] datasize;
reg [7:0] repeatNum;

integer errorFlag, ii, jj, flag;

initial $readmemb("patterns", data);

// Instantiate the Unit Under Test (UUT)
UDC DUT(.upDown(upDown), .clk(clk), .rst(rst), .sevenSegment(dataOut), .anode(anodeOut));

initial begin
	clk = 1;
	forever
		#5 clk = ~clk;
end

initial begin
	datasize  = data[0];
	errorFlag = 0;
	flag 		 = 0;
	rst       = 0;
	upDown    = 1;
	repeat(5) @(posedge clk);
	rst       <= #1 1;
	@(posedge clk);
	rst       <= #1 0;
	for(ii = 0; ii<5; ii = ii +1) begin
		jj = 0;
		repeat(datasize) @(dataOut) begin
			@(negedge clk);
			jj = jj + 1;
			if(dataOut !== data[jj]) begin
				$display("Output ERROR at time %d, expected %b, received %b", $time, data[jj], dataOut);
				errorFlag = errorFlag +1;
			end
			else
				$display("Output CORRECT at time %d, expected %b, received %b", $time, data[jj], dataOut);
		end
	end
	upDown    = 0;
	for(ii = 0; ii<5; ii = ii +1) begin
		if(flag==0) begin
			jj = 10;
			repeatNum = datasize-1;
		end else begin
			jj = 11;
			repeatNum = datasize;
		end
		flag = 1;
		repeat(repeatNum) @(dataOut) begin
			@(negedge clk);
			jj = jj - 1;
			if(dataOut !== data[jj]) begin
				$display("Output ERROR at time %d, expected %b, received %b", $time, data[jj], dataOut);
				errorFlag = errorFlag +1;
			end else
				$display("Output CORRECT at time %d, expected %b, received %b", $time, data[jj], dataOut);
		end
	end
	if(errorFlag == 0)
		$display("No ERRORs Found!");
	else
		$display("%d ERRORs Found!", errorFlag);
	$finish;
end
endmodule
