`timescale 1ns / 1ps
module calc_tb;

	// Inputs
	reg clk;
	reg rst;
	reg validIn;
	reg [7:0] dataIn;

	// Outputs
	wire [7:0] dataOut;
	
	parameter CNTSIZE = 2;

	// Instantiate the Unit Under Test (UUT)
	calc uut(.clk(clk), .rst(rst), .validIn(validIn), .dataIn(dataIn), .dataOut(dataOut));

	reg [7:0] data   [0:63];
	reg [7:0] result [0:63];
	reg [7:0] datasize, resultsize;
	integer ii, jj, errorFlag;
	
	initial $readmemh("data", data);
	initial $readmemh("result", result);

	initial begin
		clk = 1;
		forever
			#5 clk = ~clk;
	end
	
	initial begin
		datasize = data[0];
		rst = 0;
		validIn = 0;
		dataIn = 0;
		repeat(20) @(posedge clk);
		rst <= #1 1;

		@(posedge clk);
		rst <= #1 0;

		repeat(2) begin
			ii = 0;
			repeat(datasize) begin
				ii = ii +1;
				dataIn <= #1 data[ii];
				validIn <= #1 1;

				@(posedge clk);
				validIn <= #1 0;
				repeat(10) @(posedge clk);
			end		
		end
	end



	initial begin
		resultsize = result[0];
		errorFlag = 0;
		repeat(2) begin
			jj = 0;

			repeat(resultsize) @(dataOut) begin
				@(negedge clk);

				jj = jj +1;
				if(dataOut !== result[jj]) begin
                                        $display("Output Error at time %d, expected %d, received %d, data #%d", $time, result[jj], dataOut, jj);
					errorFlag = errorFlag +1; 
				end else
                                        $display("True Value at time %d, expected %d, received %d, data #%d", $time, result[jj], dataOut, jj);
			end

		   @(posedge clk);
		end

		if(errorFlag == 0)
			$display("Design contains no errors");
		else
			$display("Design contains %d errors!", errorFlag);

		$finish;
	end
	initial begin
		repeat(70) @(posedge clk);
		$finish;
	end

endmodule

