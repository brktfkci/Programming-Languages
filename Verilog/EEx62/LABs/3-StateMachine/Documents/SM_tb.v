module SM_tb();
reg clk, rst, btn;
integer count = 0;
integer st;
reg [2:0] refOut;
reg btn_f;

wire [2:0] designOut;

SM DUT(.clk(clk), .rst(rst), .btn(btn), .ledOut(designOut));

initial begin 
	clk = 0;
	forever
		#10 clk = ~clk;
end

initial begin 
	$dumpfile("tb.vcd");
	$dumpvars;
	rst = #1 1; 
	repeat(4) @(posedge clk);
	rst = #1 0;
	repeat(10) @(posedge clk);
	rst = #1 1; 
	repeat(2) @(posedge clk);
	rst = #1 0;
end

always @(posedge clk) begin
	btn <= #1 $random;
	btn_f <= #1 btn;
	calculateRefState;
	checkOutput;
	count <= #1 count+1;
	if (count == 1000) begin 
		$finish;
	end
end

task checkOutput;
begin
	#1
	if(~(refOut === designOut)) begin
		$display("ERROR! ==> Time: %d  btn: %b RefOut: %b DesignOut: %b",$time, btn, refOut, designOut);
		$finish;
	end else begin
		$display("CORRECT ==> Time: %d  btn: %b RefOut: %b DesignOut: %b",$time, btn, refOut, designOut);
	end 
end
endtask

task calculateRefState;
	begin
		if(rst)
			st <= #1 0;
		else
			if(btn == 1 && btn_f == 0)
				if(st == 3)
					st <= #1 0;
				else
					st <= #1 st + 1;
	end
endtask

always @(*)begin
	if(rst)
		refOut = 3'b001;
	else
		case(st)
			0: refOut = 3'b001;
			1: refOut = 3'b010;
			2: refOut = 3'b011;
			3: refOut = 3'b100;
		endcase
end

endmodule

