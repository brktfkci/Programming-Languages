`timescale 1ns / 1ps

// Company: Ozyegin University
// Engineer: Fatih Temizkan

module debounce_tb;
  reg clk,rst,in;
  wire out;

  always #3.5 clk = ~clk;

  debounce DUT(in, clk, out, rst);

initial begin
	$display("Simulation Begins...");
	initialize;
	repeat(1) @(posedge clk);
	rst <= 1;
	repeat(1) @(posedge clk);
	rst <= 0;
	repeat(4) @(posedge clk);
	repeat(10) begin
	  @(posedge clk);
	  #1 in=1'b1;
	end
	repeat(50) begin
		repeat(2) begin
		  @(posedge clk);
		  #1 in=1'b0;
		end
		repeat(2) begin
		  @(posedge clk);
		  #1 in=1'b1;
		end
	end
	repeat(100) begin
	  @(posedge clk);
	  #1 in=1'b1;
	end
	repeat(10) begin
	  @(posedge clk);
	  #1 in=1'b0;
	end
		repeat(50) begin
		repeat(2) begin
		  @(posedge clk);
		  #1 in=1'b1;
		end
		repeat(2) begin
		  @(posedge clk);
		  #1 in=1'b0;
		end
	end
	repeat(100) begin
	  @(posedge clk);
	  #1 in=1'b0;
	end
	$finish;
end

task initialize;
begin
		clk = 0;
		rst = 0;
		in = 0;
end
endtask

endmodule

