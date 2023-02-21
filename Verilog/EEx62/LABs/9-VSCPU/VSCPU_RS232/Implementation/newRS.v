`timescale 1ns / 1ps
module RStoRAM(clk, rst, rxDone, rxIn, txStart, txDone, txOut, en, addr, dataOut, dataIn, pCounter, resetMPU,freezeFlag);

parameter SIZE = 10;

// global pins
input clk, rst;

// receiver module connection pins
input rxDone;
input [7:0] rxIn;

// transmitter module connection pins
output reg txStart;
input      txDone;
output reg [7:0] txOut;

// Dual Port RAM connection pins
output reg en;
output reg [SIZE-1:0] addr;
output reg     [31:0] dataOut;
input          [31:0] dataIn;

// resetMPU to MPU
input [SIZE-1:0] pCounter;
output reg resetMPU;
output reg freezeFlag;

localparam [7:0]
      A  = 65,
      W  = 87,
      R  = 82,
      S  = 83,
		F	= 70,
		I  = 73,
		C	= 67,
		G	= 71;

wire [3:0] binary;
wire number;

reg resetMPUNext;
reg [SIZE-1:0] addrNext;
reg wFlagNext, wFlag, txStartNext;
reg [3:0] txBinary, resetFlagNext, resetFlag;
wire [7:0] txAscii;
reg [3:0] txCountNext, txCount;
reg [8:0] data0Next, data0, data1, data2, data3, data4, data5, data6, data7, data8; 
reg freezeFlagNext, stepFlag, stepFlagNext;
reg [SIZE-1:0] pCounterDly;

ascii2binary ascii2binary(.in(rxIn), .out({number, binary}));
binary2ascii binary2ascii(.in(txBinary), .out(txAscii));

always@(posedge clk)begin
	pCounterDly <= pCounter;
	stepFlag  <= stepFlagNext;
	addr      <= addrNext;
	wFlag 	 <= wFlagNext;
	txCount   <= txCountNext;
	resetMPU  <= resetMPUNext;
	resetFlag <= resetFlagNext;
	txStart 	 <= txStartNext;
	txOut     <= txAscii;
	freezeFlag<= freezeFlagNext;
	if(rst)begin
		data0 <= data0Next;
		data1 <= 0;
		data2 <= 0;
		data3 <= 0;
		data4 <= 0;
		data5 <= 0;
		data6 <= 0;
		data7 <= 0;
		data8 <= 0;
	end else if(rxDone)begin
		data0 <= data0Next;
		data1 <= data0;
		data2 <= data1;
		data3 <= data2;
		data4 <= data3;
		data5 <= data4;
		data6 <= data5;
		data7 <= data6;
		data8 <= data7;
	end
end

always@(*)begin
	data0Next = data0;
	if(rst)begin
		data0Next = 0;
	end else begin
		if(rxDone)begin
			if((rxIn== A) || (rxIn == W) || (rxIn == S) || (rxIn == R) || (rxIn == F) || (rxIn == C) || (rxIn == G) || (rxIn == I))
				data0Next = {1'b0, rxIn};
			else
				data0Next = {number, 4'b0000, binary};
		end
	end
end

always@(*)begin
	resetMPUNext = resetMPU;
	addrNext = addr;
	en = 0;
	dataOut = 0;
	wFlagNext = wFlag;
	txCountNext = txCount;
	txStartNext = txStart;
	txBinary = 0;
	resetFlagNext = resetFlag;
	freezeFlagNext = freezeFlag;
	stepFlagNext = stepFlag;
	if(rst) begin
		resetFlagNext = 1;
		txStartNext = 0;
		resetMPUNext = 1;
		addrNext = 0;
		wFlagNext = 1;
		txCountNext = 8;
		stepFlagNext = 0;
	end else if(data0[7:0] == S) begin
		if(stepFlag == 0)
			begin
			resetMPUNext = ~resetMPU;
			freezeFlagNext = 0;
			stepFlagNext = 1;
			end
	end else if(data3[7:0] == A && data2[8] && data1[8] && data0[8]) begin
		addrNext = {data2[1:0], data1[3:0], data0[3:0]};
	end else if(data8[7:0] == W && data7[8] && data6[8] && data5[8] && data4[8] && data3[8] && data2[8] && data1[8] && data0[8] && wFlag)begin
		wFlagNext = 0;
		en = 1;
		addrNext = addr + 1;
		dataOut = {data7[3:0], data6[3:0], data5[3:0], data4[3:0], data3[3:0], data2[3:0], data1[3:0], data0[3:0]};
	end else if(data0[7:0] == F) begin
		if(resetMPU == 1)
			resetMPUNext = 0;
		freezeFlagNext = 1;
	end else if(data0[7:0] == C) begin
		if(resetMPU == 1)
			resetMPUNext = 0;
		freezeFlagNext = 0;
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
	end else if(data3[7:0] == G && data2[8] && data1[8] && data0[8]) begin
		if(resetMPU == 1)
			resetMPUNext = 0;
		freezeFlagNext = 0;
		if(pCounter == {data2[1:0], data1[3:0], data0[3:0]})
			freezeFlagNext = 1;
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
	end else if(data0[7:0] == I) begin
		if(resetMPU == 1)
			resetMPUNext = 0;
		else if(stepFlag == 0)
			begin
			freezeFlagNext = 0;
			if(pCounterDly != pCounter)
				begin
				freezeFlagNext = 1;
				stepFlagNext = 1;
				end
			end

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

	end else if(data0[7:0] == R) begin	
		if(txCount==8 || txDone)begin
			txStartNext = 1;
			txCountNext = txCount - 1;
			if(txCount == 8)
				txBinary = dataIn[31:28];
			else if(txCount == 7)
				txBinary = dataIn[27:24];
			else if(txCount == 6)
				txBinary = dataIn[23:20];
			else if(txCount == 5)
				txBinary = dataIn[19:16];
			else if(txCount == 4)
				txBinary = dataIn[15:12];
			else if(txCount == 3)
				txBinary = dataIn[11:8];
			else if(txCount == 2)
				txBinary = dataIn[7:4];
			else if(txCount == 1) begin
				txBinary = dataIn[3:0];
 			end else begin
				txStartNext = 0;
				txCountNext = 0;
			end
		end
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
		if(rxDone)
			stepFlagNext = 0;
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
	end
	if(rxDone)begin
		wFlagNext = 1;
		txCountNext = 8;
		resetFlagNext = 1;
	end
end

endmodule



////////////////////////////////////////////////////////
module ascii2binary(in, out);
input [7:0] in;
output reg [4:0] out;

always@(*)begin
	case(in)
		48		: out = 5'b10000; 
		49		: out = 5'b10001;
		50		: out = 5'b10010;
		51		: out = 5'b10011;
		52		: out = 5'b10100;
		53		: out = 5'b10101;
		54		: out = 5'b10110;
		55 		: out = 5'b10111;
		56 		: out = 5'b11000;
		57 		: out = 5'b11001;
		97 		: out = 5'b11010;
		98 		: out = 5'b11011;
		99 		: out = 5'b11100;
		100		: out = 5'b11101;
		101		: out = 5'b11110;
		102		: out = 5'b11111;
		default	: out = 5'b00000;
	endcase
end
endmodule


///////////////////////////////////////////////
module binary2ascii(in, out);
input [3:0] in;
output reg [7:0] out;

always@(*)begin
	case(in)
	0		: out = 48;
	1		: out = 49;
	2		: out = 50;
	3		: out = 51;
	4		: out = 52;
	5		: out = 53;
	6		: out = 54;
	7 		: out = 55;
	8 		: out = 56;
	9 		: out = 57;
	10 	: out = 97;
	11 	: out = 98;
	12 	: out = 99;
	13		: out = 100;
	14		: out = 101;
	15		: out = 102;
	endcase
end
endmodule
