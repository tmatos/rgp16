
///////////////////////////////////////////////////////////////////////////////
// execution - terceira etapa do pipeline
///////////////////////////////////////////////////////////////////////////////

`ifndef _EX_V_

`define _EX_V_

`include "constants.v"

///////////////////////////////////////////////////////////////////////////////
// ULA
///////////////////////////////////////////////////////////////////////////////
module ula(a, b, result, /*carryout, over,*/ aluop);
   input [15:0] a;
   input [15:0] b;
   input [3:0] aluop;
   output reg [15:0] result;
   //output carryout;
   //output over;
   
   always @(a, b, aluop) begin
      case (aluop)
         4'h0: result <= a & b;
         4'h1: result <= a | b;
         4'h5: result <= a + b;
         4'h6: result <= a - b;
         4'h9: result <= a < b ? 16'h0001 : 16'h0000; 
         // ...
         default: result <= 0;
      endcase
   end
   
endmodule

///////////////////////////////////////////////////////////////////////////////
// ULA control
///////////////////////////////////////////////////////////////////////////////
module ula_control(opcode_in, ula_amuxsel_out, ula_bmuxsel_out, ulaop_out, clk);
   input [7:0] opcode_in;
   output reg ula_amuxsel_out; // seleciona operando a da ula, 0 para a, 1 para 0x0000
   output reg ula_bmuxsel_out; // seleciona operando b da ula, 0 para b, 1 para imediato
   output reg [3:0] ulaop_out; // seleciona qual operacao da ula
   input clk;
   
   always @( posedge clk ) begin //opcode_in) begin
      if (opcode_in == `AND) begin
         ulaop_out <= 4'h0;
         ula_amuxsel_out <= 0;
         ula_bmuxsel_out <= 0;
      end
      else if (opcode_in == `OR) begin
         ulaop_out <= 4'h1;
         ula_amuxsel_out <= 0;
         ula_bmuxsel_out <= 0;
      end
      if (opcode_in == `ADD) begin
         ulaop_out <= 4'h5;
         ula_amuxsel_out <= 0;
         ula_bmuxsel_out <= 0;
      end
      else if (opcode_in == `LW) begin
         ulaop_out <= 4'h5;  ///??? louk
         ula_amuxsel_out <= 0; ///??? louk
         ula_bmuxsel_out <= 0; ///??? louk
      end
      else if (opcode_in == `LW1) begin
         ulaop_out <= 4'h5;
         ula_amuxsel_out <= 1;
         ula_bmuxsel_out <= 1;
      end
      else begin // um default
         ulaop_out <= 4'h0;
         ula_amuxsel_out <= 0;
         ula_bmuxsel_out <= 0;
      end
   end
   
   // rascunho...
   
endmodule

`endif // _EX_V_
