`timescale 1ns / 1ps
module MPU(clk, rst, data_fromRAM, wrEn, addr_toRAM, data_toRAM, freezeFlag, pCounter);

parameter SIZE = 10;

input clk, rst;
input wire [31:0] data_fromRAM;
output reg wrEn;
output reg [SIZE-1:0] addr_toRAM;
output reg [31:0] data_toRAM;
input freezeFlag;
output reg [SIZE-1:0] pCounter;



// internal signals
reg [ 3:0] opcode, opcodeNext;
reg [13:0] operand1, operand2, operand1Next, operand2Next;
reg [SIZE-1:0] /*pCounter,*/ pCounterNext;
reg [31:0] num1, num2, num1Next, num2Next;
reg [ 2:0] state, stateNext;

reg [SIZE-1:0] addr_toRAMReg;
reg [31:0] data_toRAMReg;

always @(posedge clk)
begin
if(!freezeFlag) begin
	state    <= stateNext;
	pCounter <= pCounterNext;
end
else
	state <=  0;
opcode   <= opcodeNext;
operand1 <= operand1Next;
operand2 <= operand2Next;
num1     <= num1Next;
num2     <= num2Next;
addr_toRAMReg <= addr_toRAM;
data_toRAMReg <= data_toRAM;
end

always @*
begin
pCounterNext = pCounter;
addr_toRAM = addr_toRAMReg;
opcodeNext = opcode;
operand1Next = operand1;
operand2Next = operand2;
wrEn = 0;
data_toRAM = data_toRAMReg;
stateNext = state;
num1Next = num1;
num2Next = num2;
if(rst)
	begin
	addr_toRAM = 0;
	pCounterNext = 0;
	stateNext = 0;
	opcodeNext = 0;
	operand1Next = 0;
	operand2Next = 0;
	wrEn = 0;
	data_toRAM = 0;
	num1Next = 0;
	num2Next = 0;
	end
else if(state == 0)                       // take instruction
	begin
	addr_toRAM = pCounter;
	stateNext = 1;
	end
else if(state == 1)                       // instruction taken, take one number from RAM
	begin
	opcodeNext   = data_fromRAM[31:28];
	operand1Next = data_fromRAM[27:14];
	operand2Next = data_fromRAM[13: 0];
	case(opcodeNext)
		4'b0000, 4'b0001, 4'b0010, 4'b0011, 4'b0100, 4'b0101, 4'b0110, 4'b0111, 4'b1001, 4'b1011, 4'b1101, 4'b1110, 4'b1111: addr_toRAM = operand1Next;
		4'b1000, 4'b1010, 4'b1100: addr_toRAM = operand2Next;
	endcase
	if(opcodeNext == 4'b1001)
		begin
		wrEn = 1;
		data_toRAM = operand2Next;
		pCounterNext = pCounter + 1;
		end
	case(opcodeNext)
		4'b1001: stateNext = 0;
		4'b0000, 4'b0010, 4'b0100, 4'b0110, 4'b1010, 4'b1011, 4'b1100, 4'b1110: stateNext = 2;
		4'b0001, 4'b0011, 4'b0101, 4'b0111, 4'b1000, 4'b1101, 4'b1111: stateNext = 3;
	endcase
	end
else if(state == 2)        					// first number is taken, take another number from RAM
	begin
	num1Next = data_fromRAM;
	case(opcodeNext)
		4'b0000, 4'b0010, 4'b0100, 4'b0110, 4'b1011, 4'b1110: addr_toRAM = operand2Next;
		4'b1010: addr_toRAM = data_fromRAM;
		4'b1100: addr_toRAM = operand1Next;
	endcase
//	stateNext = 3;
if(opcodeNext == 4'b1100 && data_fromRAM != 0)
begin
stateNext = 0;
pCounterNext = pCounter + 1;
end
else
stateNext = 3;
	end
else if(state == 3)            // write back
	begin
	num2Next = data_fromRAM;
	stateNext = 0;
	case(opcodeNext)
	4'b0000, 4'b0001, 4'b0010, 4'b0011, 4'b0100, 4'b0101, 4'b0110, 4'b0111, 4'b1000, 4'b1010, 4'b1110, 4'b1111: addr_toRAM = operand1Next;
	4'b1011: addr_toRAM = num1Next;
	endcase
	case(opcodeNext)
		4'b0000: data_toRAM = num1Next + num2Next;
		4'b0001: data_toRAM = num2Next + operand2;
		4'b0010: data_toRAM = ~(num1Next & num2Next);
		4'b0011: data_toRAM = ~(num2Next & operand2);
		4'b0100: begin if(num2Next < 32) data_toRAM = num1Next >> num2Next; else data_toRAM = num1Next << (num2Next - 32); end
		4'b0101: begin if(operand2 < 32) data_toRAM = num2Next >> operand2; else data_toRAM = num2Next << (operand2 - 32); end
		4'b0110: begin if(num1Next < num2Next) data_toRAM = 1; else data_toRAM = 0; end
		4'b0111: begin if(num2Next < operand2) data_toRAM = 1; else data_toRAM = 0; end
		4'b1000: data_toRAM = num2Next;
		4'b1001: data_toRAM = operand2;
		4'b1010: data_toRAM = num2Next;
		4'b1011: data_toRAM = num2Next;
//		4'b1110: data_toRAM = num1Next * num2Next; //Design is too large for the given device so this line is commented out
//		4'b1111: data_toRAM = num2Next * operand2; //Design is too large for the given device so this line is commented out
	endcase
	case(opcodeNext)
		4'b1100, 4'b1101: wrEn = 0;
		default: wrEn = 1;
	endcase
	case(opcodeNext)
		4'b1100: begin if(num1Next == 0) pCounterNext = num2Next; else pCounterNext = pCounter + 1; end
		4'b1101: pCounterNext = data_fromRAM + operand2;
		default: pCounterNext = pCounter + 1;
	endcase
	end
end

endmodule
