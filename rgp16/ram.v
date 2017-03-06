
///////////////////////////////////////////////////////////////////////////////
// a ram (kkk)
///////////////////////////////////////////////////////////////////////////////

`ifndef _RAM_V_

`define _RAM_V_

///////////////////////////////////////////////////////////////////////////////
// RAM (mimics)
///////////////////////////////////////////////////////////////////////////////
module ram(
    output reg [15:0] data_out,
    input wire [15:0] addr_in,
    input wire [15:0] data_in, 
    input wire write_enable,
    input wire clk
   );

   reg [15:0] mem [0:511];

   // programinha pre carregado pra teste
   initial
   begin
      //            |--||--||--||--|
      mem[0] <= 16'b1010000100100000 ; // LW A0, 123
      mem[1] <= 16'b0000000001111011 ; // o 123 acima
      mem[2] <= 16'b0000111100000000 ; // NOP
      mem[3] <= 16'b0000111100000000 ; // NOP
      mem[4] <= 16'b0000111100000000 ; // NOP
      mem[5] <= 16'b0100001100110010 ; // ADD A1, A0
      mem[6] <= 16'b1100001000110000 ; // SW A1, 10(ZERO)
      mem[7] <= 16'b0000000000001101 ; // o 10 acima
      mem[8] <= 16'b1100000100110010 ; // LW A1, 64(A0)
      mem[9] <= 16'b0000000001000000 ; // o 64 acima
      mem[10] <= 16'b0000111100000000 ; // NOP
      mem[11] <= 16'b1010000101010000 ; // LW A3, 0
      mem[12] <= 16'b0000000000000000 ; // o 0 acima
      mem[13] <= 16'b0000111100000000 ; // NOP
      mem[14] <= 16'b0000111100000000 ; // NOP
      mem[15] <= 16'b0000111100000000 ; // NOP
      mem[16] <= 16'b0000111100000000 ; // NOP
      mem[17] <= 16'b0010101101010000 ; // JR A3
      //mem[18] <= 
      //mem[19] <= 
   end

   always @(posedge clk) begin
      if (write_enable) mem[addr_in] <= data_in;
   end

   always @(posedge clk) begin
      data_out <= mem[addr_in];
   end
   
endmodule

`endif // _RAM_V_
