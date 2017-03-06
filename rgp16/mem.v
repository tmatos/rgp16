
///////////////////////////////////////////////////////////////////////////////
// memory - quarta etapa do pipeline
///////////////////////////////////////////////////////////////////////////////

`ifndef _MEM_V_

`define _MEM_V_

`include "constants.v"

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////
module branch_select(opcode_in, set_jump_out, clk);
   input [7:0] opcode_in;
   output reg set_jump_out;
   input clk;

   always @(posedge clk) begin
      case(opcode_in)
         `JR: set_jump_out <= 1;
         //`BRFL: RFlags == Imediate ? vai pra Result; // TODO
         //`CALL: pula para result, mas salva o reg RA; // TODO
         //`RET: pula para o RA; // TODO
         default: set_jump_out <= 0;
      endcase
   end

   
endmodule

`endif // _MEM_V_
