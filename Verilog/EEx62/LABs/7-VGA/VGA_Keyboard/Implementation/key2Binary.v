module key2Binary
   (
    input wire [7:0] key_code,
    output reg [4:0] binary
   );

always @*
   case(key_code)
       8'h45: binary = 0;   // 0
       8'h16: binary = 1;   // 1
       8'h1e: binary = 2;   // 2
       8'h26: binary = 3;   // 3
       8'h25: binary = 4;   // 4
       8'h2e: binary = 5;   // 5
       8'h36: binary = 6;   // 6
       8'h3d: binary = 7;   // 7
       8'h3e: binary = 8;   // 8
       8'h46: binary = 9;   // 9
       8'h1c: binary = 10;   // A
       8'h32: binary = 11;   // B
       8'h21: binary = 12;   // C
       8'h23: binary = 13;   // D
       8'h24: binary = 14;   // E
       8'h2b: binary = 15;   // F
       default: binary = 16; // 
    endcase

endmodule
