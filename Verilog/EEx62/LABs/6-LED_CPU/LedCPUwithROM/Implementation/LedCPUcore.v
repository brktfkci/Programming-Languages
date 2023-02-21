module LedCPUcore( 
	clk,
	rst,
	addrRd,
	dataRd,
	outPattern
    );
input clk,rst; 
output reg [7:0] addrRd;
input [15:0] dataRd;
output reg [7:0] outPattern; 

parameter FREQ_DELAY = 50_000_000/256;

reg [7:0] addrRdNext;
reg state, stateNext;
reg [7:0] outPatternNext;
reg [7:0] processTime, processTimeNext;
reg [17:0] count, countNext;

always@(posedge clk) begin
	addrRd      <= #1 addrRdNext;
	state       <= #1 stateNext;
	outPattern  <= #1 outPatternNext;
	processTime <= #1 processTimeNext;
	count       <= #1 countNext;
end

always@(*)begin
	outPatternNext  = outPattern;
	processTimeNext = processTime;
	      stateNext = state;
	     addrRdNext = addrRd;
	      countNext = count;
	if(rst)begin
		   addrRdNext = 0;
			stateNext = 0;
       outPatternNext = 0;
      processTimeNext = 0;
			countNext = 0;
	end
	else begin
		case(state)
		0:begin
			if(dataRd[7:0] == 0)begin
				addrRdNext = dataRd[15:8];
			end 
			else begin
				stateNext = 1;
			end
		end
		1:begin
			countNext = count + 1;
			if(count == FREQ_DELAY)begin
				outPatternNext = dataRd[15:8];
				processTimeNext = processTime + 1;
				countNext = 0;
				if(processTime == dataRd[7:0])begin
					addrRdNext = addrRd + 1;
					processTimeNext = 0;
					stateNext = 0;
				end
			end
		end
		endcase
	end
end
endmodule
