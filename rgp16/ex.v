
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
         4'h0: result <= 0; // NOP, ou NADA
         4'h1: result <= a & b; // AND
         4'h2: result <= a | b; // OR
         4'h3: result <= 0;  // CMP, TODO: set RFlags !!!!!
         4'h4: result <= ~a; // NOT
         4'h5: result <= a + b; // ADD
         4'h6: result <= a - b; // SUB
         4'h7: result <= a * b; // MUL, TODO: (R0 como auxliar)
         4'h8: result <= a / b; // DIV, TODO: (R0 como auxliar)
         4'hA: result <= a + b; // para LW e LW1 TODO: tratar segfault
         ///4'hB: result <= a < b ? 16'h0001 : 16'h0000;
         // ...
         default: result <= 0;
      endcase
   end
   
endmodule

///////////////////////////////////////////////////////////////////////////////
// ULA control
///////////////////////////////////////////////////////////////////////////////
module ula_control(opcode_in, ula_opsmuxsel_out, ula_resmuxsel_out, ulaop_out, clk);
   input [7:0] opcode_in;
   output reg ula_opsmuxsel_out; // seleciona operandos da ula, 0 para a e b, 1 para b e imed
   output reg ula_resmuxsel_out; // seleciona o resultado final da unidade, 0 para ula, 1 para imediato
   output reg [3:0] ulaop_out; // seleciona qual operacao da ula
   input clk;
   
   always @( posedge clk ) begin //opcode_in) begin
      if (opcode_in == `NOP) begin
         ulaop_out <= 4'h0;
         ula_opsmuxsel_out <= 0;
         ula_resmuxsel_out <= 0;
      end
      if (opcode_in == `AND) begin
         ulaop_out <= 4'h1;
         ula_opsmuxsel_out <= 0;
         ula_resmuxsel_out <= 0;
      end
      else if (opcode_in == `OR) begin
         ulaop_out <= 4'h2;
         ula_opsmuxsel_out <= 0;
         ula_resmuxsel_out <= 0;
      end
      else if (opcode_in == `CMP) begin
         ulaop_out <= 4'h3;
         ula_opsmuxsel_out <= 0;
         ula_resmuxsel_out <= 0;
      end
      else if (opcode_in == `NOT) begin
         ulaop_out <= 4'h4;
         ula_opsmuxsel_out <= 0;
         ula_resmuxsel_out <= 0;
      end
      if (opcode_in == `ADD) begin
         ulaop_out <= 4'h5;
         ula_opsmuxsel_out <= 0;
         ula_resmuxsel_out <= 0;
      end
      else if (opcode_in == `SUB) begin
         ulaop_out <= 4'h6;
         ula_opsmuxsel_out <= 0;
         ula_resmuxsel_out <= 0;
      end
      else if (opcode_in == `MUL) begin
         ulaop_out <= 4'h7;
         ula_opsmuxsel_out <= 0;
         ula_resmuxsel_out <= 0;
      end
      else if (opcode_in == `DIV) begin
         ulaop_out <= 4'h8;
         ula_opsmuxsel_out <= 0;
         ula_resmuxsel_out <= 0;
      end
      else if (opcode_in == `LW) begin
         ulaop_out <= 4'hA;
         ula_opsmuxsel_out <= 1; // B + imed
         ula_resmuxsel_out <= 0; // ula result
      end
      else if (opcode_in == `LW1) begin
         ulaop_out <= 4'hA;
         ula_opsmuxsel_out <= 1;
         ula_resmuxsel_out <= 1; // take imed as result
      end
      else begin // um default
         ulaop_out <= 4'h0;
         ula_opsmuxsel_out <= 0;
         ula_resmuxsel_out <= 0;
      end
   end
   
   // rascunho ainda...
   
endmodule

module address_adder(base_in, incr_in, addr_out);
   input [15:0] base_in;
   input [15:0] incr_in;
   output reg [15:0] addr_out;
   
   always @(base_in, incr_in) begin
      addr_out <= (base_in + incr_in);
   end
endmodule

`endif // _EX_V_
