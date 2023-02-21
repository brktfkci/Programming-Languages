`timescale 1ns / 1ps
module rotateDigit(in0, in1, in2, in3, in4, in5, in6, state, seg0, seg1, seg2, seg3);

input [4:0] in0, in1, in2, in3, in4, in5, in6;
input [2:0] state;
output reg [4:0] seg0, seg1, seg2, seg3;

always @* begin
	seg0 = 0;
	seg1 = 0;
	seg2 = 0;
	seg3 = 0;
	case(state)
		0:begin
			seg0 = in0;
			seg1 = in1;
			seg2 = in2;
			seg3 = in3;
		end
		1:begin
			seg0 = in6;
			seg1 = in0;
			seg2 = in1;
			seg3 = in2;
		end
		2:begin
			seg0 = in5;
			seg1 = in6;
			seg2 = in0;
			seg3 = in1;
		end
		3:begin
			seg0 = in4;
			seg1 = in5;
			seg2 = in6;
			seg3 = in0;
		end
		4:begin
			seg0 = in3;
			seg1 = in4;
			seg2 = in5;
			seg3 = in6;
		end
		5:begin
			seg0 = in2;
			seg1 = in3;
			seg2 = in4;
			seg3 = in5;
		end
		6:begin
			seg0 = in1;
			seg1 = in2;
			seg2 = in3;
			seg3 = in4;
		end	
	endcase
end

endmodule
