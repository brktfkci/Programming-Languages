//=================================================================================================
// mas_pi_cont module declaration
//=================================================================================================

module mass_block
#(

    // Specifies the width of multiplication result
    parameter ADD_WIDTH = 44,

    // Specifies the width all i/o ports
    parameter IO_WIDTH = 18,

    // Specifies the number of sys_clk_i delays required before the mas done signal is asserted.
    parameter CYCLE_NUM = 2
  )
  (
   // system reset
   input reset_i,

   // system sys_clk_i
   input sys_clk_i,

   // Control Input to enable mas block
   input mas_en_i,

   // Control Input to select addition/subtraction operation
   input sub_i,

   // Data Input for first operand of multiplication
   input signed [(IO_WIDTH - 1):0] mul_a_i,

   // Data Input for second operand of multiplication
   input signed [(IO_WIDTH - 1):0] mul_b_i,

   // Data input for addition/subtraction operation
   input signed [(ADD_WIDTH - 1):0] add_c_i,

   // Data Output of mas block
   output reg   [(ADD_WIDTH - 1):0] product_o,

   // Control Output validating the data output of mas block
   output reg mas_done_o
);
   reg[2:0] mas_done_sync; 
   reg[2:0] mas_done_sync_next; 
   reg [(ADD_WIDTH - 1):0] product_o_next;
   reg mas_done_o_next;
   

//-----------------------------------------------------------------------
// Name       : MCP_0_GEN
// Description: Output generated w.r.t mas_en_i occurrence
//------------------------------------------------------------------------  
    generate
		if (CYCLE_NUM == 0) begin : MCP_0_GEN   
	  
			always @(posedge sys_clk_i) begin
				mas_done_o 	  <= #1 mas_done_o_next;
				mas_done_sync <= #1 mas_done_sync_next;  
			end
			
			always @(*) begin : SINGLE_MCYCLE_PATHS_PROC
				mas_done_o_next    = mas_done_o;
				mas_done_sync_next = mas_done_sync;
				if (reset_i == 1'b1) begin
					mas_done_o_next    = 1'b0; 
					mas_done_sync_next = 1'b0;
				end 
				else begin
					mas_done_o_next	= mas_en_i; 
				end
			end
		end
    endgenerate 
//------------------------------------------------------------------------
// Name       : MCP_1_GEN
// Description: Output generated after 1 sys_clk delay from the 
//              occurrence of mas_en_i
//------------------------------------------------------------------------
	generate
		if (CYCLE_NUM == 1) begin : SINGLE_MCYCLE_PATHS_ZEN
		
			always @(posedge sys_clk_i) begin
				mas_done_o <= #1 mas_done_o_next;
			end
			
			always @(*) begin : SINGLE_MCYCLE_PATHS_PROC
				mas_done_o_next = mas_done_o;
				if (reset_i == 1'b1) begin
					mas_done_o_next	= 1'b0; 
				end 
				else begin
					mas_done_o_next	= mas_en_i; 
				end 
			end 
		end
	endgenerate 
//------------------------------------------------------------------------
// Name       : MCP_2_GEN
// Description: Output generated after 2 sys_clks delay from the 
//              occurrence of mas_en_i
//------------------------------------------------------------------------
   generate
		if (CYCLE_NUM == 2) begin : TWO_MCYCLE_PATHS_ZEN

			always @(posedge sys_clk_i) begin
				mas_done_o       <= #1 mas_done_o_next;
				mas_done_sync[0] <= #1 mas_done_sync_next[0];
			end
			
			always @(*) begin : MCP_2_GEN
				mas_done_o_next       = mas_done_o;
				mas_done_sync_next[0] = mas_done_sync[0];
				if (reset_i == 1'b1) begin
					mas_done_o_next       = 1'b0; 
					mas_done_sync_next[0] = 1'b0; 
				end 
				else begin
					mas_done_sync_next[0] = mas_en_i; 
					mas_done_o_next	      = mas_done_sync[0]; 
				end 
			end 
		end
   endgenerate 
//------------------------------------------------------------------------
// Name       : MCP_3_GEN
// Description: Output generated after 3 sys_clks delay from the 
//              occurrence of mas_en_i
//------------------------------------------------------------------------
	generate
		if (CYCLE_NUM == 3) begin : THREE_MCYCLE_PATHS_ZEN

			always @(posedge sys_clk_i) begin
				mas_done_o       <= #1 mas_done_o_next;
				mas_done_sync[0] <= #1 mas_done_sync_next[0];
				mas_done_sync[1] <= #1 mas_done_sync_next[1];
			end
			
			always @(*) begin : MCP_3_GEN
				mas_done_o_next       = mas_done_o;
				mas_done_sync_next[0] = mas_done_sync[0];
				mas_done_sync_next[1] = mas_done_sync[1];
				if (reset_i == 1'b1) begin
				    mas_done_o_next		  = 1'b0; 
				    mas_done_sync_next[0] = 1'b0; 
				    mas_done_sync_next[1] = 1'b0; 
				end 
				else begin
					mas_done_sync_next[0] = mas_en_i; 
					mas_done_sync_next[1] = mas_done_sync[0]; 
					mas_done_o_next		  = mas_done_sync[1]; 
				end 
			end 
		end
	endgenerate 
//------------------------------------------------------------------------
// Name       : MCP_4_GEN
// Description: Output generated after 4 sys_clks delay from the 
//              occurrence of mas_en_i
//------------------------------------------------------------------------
	generate
		if (CYCLE_NUM == 4) begin : FOUR_MCYCLE_PATHS_ZEN

			always @(posedge sys_clk_i) begin
				mas_done_o       <= #1 mas_done_o_next;
				mas_done_sync[0] <= #1 mas_done_sync_next[0];
				mas_done_sync[1] <= #1 mas_done_sync_next[1];
				mas_done_sync[2] <= #1 mas_done_sync_next[2];
			end
			
			always @(*) begin : MCP_4_GEN
				mas_done_o_next	      = mas_done_o;
				mas_done_sync_next[0] = mas_done_sync[0];
				mas_done_sync_next[1] = mas_done_sync[1];
				mas_done_sync_next[2] = mas_done_sync[2];
				if (reset_i == 1'b1) begin
					mas_done_o_next	        = 1'b0; 
					mas_done_sync_next[0]	= 1'b0; 
					mas_done_sync_next[1]	= 1'b0; 
					mas_done_sync_next[2]	= 1'b0; 
				end 
				else begin
					mas_done_sync_next[0] = mas_en_i; 
					mas_done_sync_next[1] = mas_done_sync[0]; 
					mas_done_sync_next[2] = mas_done_sync[1]; 
					mas_done_o_next	      = mas_done_sync[2]; 
				end 
			end 
		end
	endgenerate 

//------------------------------------------------------------------------
// Name       : MAS_PROC
// Description: Logic implemented to infer Multiplier-Adder-Subtractor
//              with registered outputs
//------------------------------------------------------------------------
	always @(posedge sys_clk_i) begin
		product_o <= #1 product_o_next;
	end
	
	always @(*)	begin : MAS_PROC
		product_o_next	= product_o;
		if (reset_i == 1'b1) begin
			product_o_next	= {ADD_WIDTH{1'b0}}; 
		end 
		else begin
			if (mas_en_i == 1'b1) begin
				if (sub_i == 1'b0) begin
					product_o_next = (add_c_i + (mul_a_i * mul_b_i)); 
				end 
				else begin
					product_o_next = (add_c_i - (mul_a_i * mul_b_i)); 
				end 
			end 
		end 
	end 
endmodule