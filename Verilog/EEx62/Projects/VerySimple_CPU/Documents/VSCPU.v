`timescale 1ns / 1ps
module VerySimpleCPU(clk, rst, data_fromRAM, wrEn, addr_toRAM, data_toRAM, pCounter);

parameter SIZE = 14;

input clk, rst;
input [31:0] data_fromRAM;
output reg wrEn;
output reg [SIZE-1:0] addr_toRAM;
output reg [31:0] data_toRAM;
output reg [SIZE-1:0] pCounter;

// internal signals
reg [ 3:0] opcode, opcodeNext;
reg [13:0] operand1, operand2, operand1Next, operand2Next;
reg [SIZE-1:0] /*pCounter,*/ pCounterNext;
reg [31:0] num1, num2, num1Next, num2Next;
reg [ 7:0] state, stateNext;

always @(posedge clk) begin
	state    <= #1 stateNext;
	pCounter <= #1 pCounterNext;
	opcode   <= #1 opcodeNext;
	operand1 <= #1 operand1Next;
	operand2 <= #1 operand2Next;
	num1     <= #1 num1Next;
	num2     <= #1 num2Next;
end

always @* begin
	stateNext    = state;
	pCounterNext = pCounter;
	opcodeNext   = opcode;
	operand1Next = operand1;
	operand2Next = operand2;
	num1Next     = num1;
	num2Next     = num2;
	addr_toRAM   = 0;
	wrEn         = 0;
	data_toRAM   = 0;
	if(rst) begin
		stateNext    = 0;
		pCounterNext = 0;
		opcodeNext   = 0;
		operand1Next = 0;
		operand2Next = 0;
		num1Next     = 0;
		num2Next     = 0;
		addr_toRAM   = 0;
		wrEn         = 0;
		data_toRAM   = 0;
	end else begin
		case(state)                       
			0: begin				  // take instruction
				operand1Next = 0;
				operand2Next = 0;
				addr_toRAM   = pCounter;
				num1Next     = 0;
				num2Next     = 0;
				stateNext    = 1;
			end 
			1:begin                   // take *A
				opcodeNext   = {data_fromRAM[28], data_fromRAM[31:29]};
				operand1Next = data_fromRAM[27:14];
				operand2Next = data_fromRAM[13: 0];
				addr_toRAM   = data_fromRAM[27:14];
				num1Next     = 0;
				num2Next     = 0;
				if(opcodeNext == 4'b0000 || opcodeNext == 4'b0110 || opcodeNext == 4'b0100
					|| opcodeNext == 4'b0111 || opcodeNext == 4'b0011 || opcodeNext == 4'b0001
					|| opcodeNext == 4'b0010 || opcodeNext == 4'b0101 || opcodeNext == 4'b1101)
					stateNext = 2;
				else if(opcodeNext == 4'b1000)
					stateNext = 6;
				else if(opcodeNext == 4'b1110)
					stateNext = 5;
				else if (opcodeNext == 4'b1100)
					stateNext = 9;
				else if (opcodeNext == 4'b1111)
					stateNext = 11;
				else if (opcodeNext == 4'b1011)
					stateNext = 13;
				else if (opcodeNext == 4'b1001)
					stateNext = 15;
				else if (opcodeNext == 4'b1010)
					stateNext = 17;
			end
			2: begin         // read B
				addr_toRAM   = operand2;
				num1Next     = data_fromRAM;
				num2Next     = 0;
				if (opcodeNext == 4'b0000)
					stateNext = 3;
				else if (opcodeNext == 4'b0110)
					stateNext = 7;
				else if (opcodeNext == 4'b0100)
					stateNext = 8;
				else if (opcodeNext == 4'b0111)
					stateNext = 10;
				else if (opcodeNext == 4'b0011)
					stateNext = 12;
				else if (opcodeNext == 4'b0001)
					stateNext = 14;
				else if (opcodeNext == 4'b0010)
					stateNext = 16;
				else if (opcodeNext == 4'b0101)
					stateNext = 18;
				else if (opcodeNext == 4'b1101)
					stateNext = 20;
			end
			3: begin            
				pCounterNext = pCounter + 1;
				opcodeNext = opcode;	
				addr_toRAM = operand1;
				num1Next = num1;
				num2Next = data_fromRAM;
				wrEn = 1;
				if(opcodeNext == 4'b0000)
					data_toRAM = num1 + data_fromRAM;
				stateNext = 0;
			end
			5: begin
				pCounterNext = data_fromRAM + operand2;
				addr_toRAM   = operand1;	
				num1Next     = data_fromRAM;
				num2Next     = operand2;
				data_toRAM   = 32'hFFFF_FFFF;
				stateNext = 0;
			end
			6: begin // *A = *A + B
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				wrEn         = 1;
				data_toRAM   = data_fromRAM + operand2;
				stateNext    = 0;
			end
			7: begin // if (*B == 0) pCounterNext = *A else pCounterNext = pCounter +1;
				pCounterNext = pCounter + 1;
				num2Next     = data_fromRAM;
				if (num2Next == 0)
					pCounterNext = num1Next;
				stateNext    = 0;
			end
			8: begin // *A = *B
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				wrEn         = 1;
				data_toRAM   = data_fromRAM;
				stateNext    = 0;
			end
			9: begin // *A = B
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				wrEn         = 1;
				data_toRAM   = operand2Next;
				stateNext    = 0;
			end
			10: begin // *A = (*A)*(*B)
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				num2Next     = data_fromRAM;
				wrEn         = 1;
				data_toRAM   = num1Next * num2Next;
				stateNext    = 0;
			end
			11: begin // *A = (A*)*(B)
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				num1Next     = data_fromRAM;
				wrEn         = 1;
				data_toRAM   = num1Next * operand2Next;
				stateNext    = 0;
			end
			12: begin // *A = *A < *B
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				num2Next     = data_fromRAM;
				wrEn         = 1;
				data_toRAM   = num1Next < num2Next;
				stateNext    = 0;
			end
			13: begin // *A = A* < B
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				num1Next     = data_fromRAM;
				wrEn         = 1;
				data_toRAM   = num1Next < operand2Next;
				stateNext    = 0;
			end
			14: begin // *A = ~(*A & *B)
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;			
				num2Next     = data_fromRAM;
				wrEn         = 1;
				data_toRAM   = ~(num1Next & num2Next);
				stateNext    = 0;
			end
			15: begin // *A = ~(A* & B)
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				num1Next     = data_fromRAM;
				wrEn         = 1;
				data_toRAM   = ~(num1Next & operand2Next);
				stateNext    = 0;
			end
			16: begin // if(*B<32) *A = A* >> *B else *A = A* << *B-32
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				num2Next     = data_fromRAM;
				wrEn         = 1;
				if (num2Next < 32) 
					data_toRAM = num1Next >> num2Next;
				else 
					data_toRAM = num1Next << (num2Next-32);
				stateNext    = 0;
			end
			17: begin // if(B<32) *A = A* >> B else *A = A* << B-32
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				num1Next     = data_fromRAM;
				wrEn         = 1;
				if (operand2Next < 32) 
					data_toRAM = num1Next >> operand2Next;
				else 
					data_toRAM = num1Next << (operand2Next-32);
				stateNext    = 0;
			end
			18: begin // read *B
				num2Next     = data_fromRAM;
				addr_toRAM   = num2Next;
				stateNext    = 19;
			end
			19: begin // *A = **B
				pCounterNext = pCounter + 1;
				addr_toRAM   = operand1;
				wrEn         = 1;
				data_toRAM   = data_fromRAM;
				stateNext    = 0;
			end
			20: begin // **A = *B
				pCounterNext = pCounter + 1;
				num2Next     = data_fromRAM;
				addr_toRAM   = num1Next;
				wrEn         = 1;
				data_toRAM   = num2Next;
				stateNext    = 0;
			end
			default: begin
				stateNext    = 0;
				pCounterNext = 0;
				opcodeNext   = 0;
				operand1Next = 0;
				operand2Next = 0;
				num1Next     = 0;
				num2Next     = 0;
				addr_toRAM   = 0;
				wrEn         = 0;
				data_toRAM   = 0;
			end
		endcase
	end
end
endmodule