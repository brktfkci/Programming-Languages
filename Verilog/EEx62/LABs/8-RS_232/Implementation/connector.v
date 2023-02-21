`timescale 1ns / 1ps
module connector(clk, rst, button_enable, switch_input, receive_done, received_data, transmit_start, transmit_done, transmit_data, led_data);

// global input & outputs
input clk, rst;
input button_enable;
input [7:0] switch_input;        // input taken from 8-bit switches
output reg [7:0] led_data;       // for outputting values to the 8-bit leds.

// rx_unit input & outputs
input receive_done;				   // will be positive when data receiving is complete
input [7:0] received_data;       // data that is received from rx unit.

// tx_unit input & outputs
input transmit_done;
output reg transmit_start;       // must be positive to start transmitting data
output reg [7:0] transmit_data;  // 8-bit data that will be transmitted to PC via tx unit. 

/////////////////////////////WRITE YOUR CODE HERE/////////////////////////////////
/**********************************SIGNALS***************************************/
reg button_enable_d;
wire button_riseedge;//, sw_transmit_c;
/********************************************************************************/
/******************************DELAY REGISTERS***********************************/
always @(posedge clk) begin
	button_enable_d <= button_enable;
end
/********************************************************************************/
// creating a rise edge detecting signal from transmit enable button
assign button_riseedge = ((button_enable == 1) && (button_enable_d == 0));

// creating transmit enable signal depending on the rise edge signal
always @(posedge clk) begin
	if(button_riseedge)	// enable data from FPGA
		transmit_start <= 1;
	else					// enable loopback data from HyperTerminal
		transmit_start <= receive_done;
end

// selecting the transmitter output (FPGA switch output or loopback from HyperTerminal)
always @(posedge clk) begin
	if(button_riseedge) begin  // transmit data from FPGA
		transmit_data <= switch_input;
		led_data      <= switch_input;
	end
	else begin                 // loopback data from HyperTerminal
		transmit_data <= received_data+1;
		led_data      <= received_data;
	end
end



endmodule
