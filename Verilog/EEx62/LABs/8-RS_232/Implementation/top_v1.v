`timescale 1ns / 1ps
/*READ FIRST

	"rx" is the 1-bit input received from the RS232 port by the receiver module, receiver
module outputs 8-bit data from its "dout" pin, which is received as "rx_module_out" in
this main block. "rx_done_tick" node becomes logic 1, when an 8-bit data is successfully
received.

	"tx" is the 1-bit output sent back to RS232 port from the transmitter module, transmitter
module takes an 8-bit input from its "din" pins.

	The circuit sends ASCII equivalent of the 8-bit data sent from the switch provided that "BTN0" is
pressed, otherwise sends back the 'received character+1' from terminal.


	The input and output units of the circuit:
	INPUTS
	Switches (from SW7 to SW0) are used for inputting 8-bit ASCII characters.
	Pushbuttons (BTN3 and BTN0) BTN3 is reset button, BTN0 is for inputting data from switches to PC.
	Expansion Connector (B2) is for Rx.
	OUTPUT
	LEDS (from LD7 to LD0) show the ASCII equivalent of the characters sent from terminal or switches.
	Expansion Connector (A3) is for Tx.

// IMPORTANT
	Interaction with rx, tx and baud generation units are made through 'connector' module. Do not make
adjustments in any other module unless really necessary.

*/
module top_v1
   #( // Default setting:
      // 115,200 baud, 8 data bits, 1 stop bit, 2^2 FIFO
      parameter DBIT = 8,     // # data bits
                SB_TICK = 16, // # ticks for stop bits, 16/24/32
                              // for 1/1.5/2 stop bits
                DVSR = 27,    //163,   // baud rate divisor
                              // DVSR = 50M/(16*baud rate)
                DVSR_BIT = 8, // # bits of DVSR
                FIFO_W = 2    // # addr bits of FIFO
                              // # words in FIFO=2^FIFO_W
   )
   (
    input wire clk, reset,
    input wire rx,
	 input wire [7:0] sw_data_in,
	 input wire sw_transmit,
    output wire tx,
    output wire [7:0] rx_data_out
   );

wire [7:0] rx_module_out;
wire [7:0]din;
wire tx_start;

// SUBMODULES
   mod_m_counter #(.M(DVSR), .N(DVSR_BIT)) baud_gen_unit
      (.clk(clk), .reset(reset), .q(), .max_tick(tick));
	 
   uart_rx #(.DBIT(DBIT), .SB_TICK(SB_TICK)) uart_rx_unit
      (.clk(clk), .reset(reset), .rx(rx), .s_tick(tick),
       .rx_done_tick(rx_done_tick), .dout(rx_module_out));


   uart_tx #(.DBIT(DBIT), .SB_TICK(SB_TICK)) uart_tx_unit
      (.clk(clk), .reset(reset), .tx_start(tx_start),
       .s_tick(tick), .din(din),
       .tx_done_tick(tx_done_tick), .tx(tx));

	debounce db(sw_transmit,clk,sw_transmit_c,reset);
	
	connector connector
		(.clk(clk), .rst(reset), .button_enable(sw_transmit_c), .switch_input(sw_data_in), 
		.receive_done(rx_done_tick), .received_data(rx_module_out), 
		.transmit_start(tx_start), .transmit_done(tx_done_tick), .transmit_data(din), 
		.led_data(rx_data_out));

endmodule
