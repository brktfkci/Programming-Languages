`timescale 1ns / 1ps
module calc(clk, rst, validIn, dataIn, ledOut);
input clk, rst, validIn;
input  [7:0] dataIn;
output reg [7:0] ledOut;

reg [1:0] state, stateNext;
reg [7:0] number, numberNext, ledOutNext;
reg [2:0] operator, operatorNext;

////////////////// for clean validIn
wire valid;
wire validClean;
reg validReg;
assign valid = validClean &!validReg; 
///////////////////
debounce dbc (.clk(clk), .rst(rst), .in(validIn), .out(validClean)); //instantination
///////////////////
always @(posedge clk) begin
	validReg   <= #1 validClean;    // for clean validIn
	state      <= #1 stateNext;
	number     <= #1 numberNext;
	operator   <= #1 operatorNext;
	ledOut     <= #1 ledOutNext;
end

always @(*) begin
	stateNext    = state;
	numberNext   = number;
	operatorNext = operator;
	ledOutNext   = ledOut;
	if(rst) begin
		stateNext    = 0;
		numberNext   = 0;
		operatorNext = 0;
		ledOutNext   = 0;
	end else begin
		case(state)
			0: begin
				if(valid) begin
					stateNext   = 1;
					numberNext  = dataIn;
					ledOutNext  = dataIn;
				end else begin
					ledOutNext = ledOut;
				end
			end
			1: begin
				operatorNext = dataIn;
				if(valid)begin
					if(operator == 3 || operator == 4 || operator == 5)begin
						stateNext = 0;
						case(operator)
						3: ledOutNext = number * number;
						4: ledOutNext = number + 1;
						5: ledOutNext = number - 1;
						endcase
					end else if(operator == 0 || operator == 1 || operator == 2)begin
						stateNext  = 2;
						ledOutNext = operator;
					end
				end else begin
					ledOutNext = number;
				end
			end
			2: begin
				if(valid) begin
					stateNext = 0;
					case(operator)
						0: ledOutNext = number * dataIn;
						1: ledOutNext = number + dataIn;
						2: ledOutNext = number - dataIn;
					endcase
				end else begin
					ledOutNext = operator;
				end
			end	
		endcase
	end
end
endmodule
