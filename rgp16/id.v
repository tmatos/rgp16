
///////////////////////////////////////////////////////////////////////////////
// instruction decode - segunda etapa do pipeline
///////////////////////////////////////////////////////////////////////////////

`ifndef _ID_V_

`define _ID_V_

`include "constants.v"

///////////////////////////////////////////////////////////////////////////////
// modulo de Controle geral
///////////////////////////////////////////////////////////////////////////////
module control(instruc_in,
               read_reg_out,
               write_reg_out,
               set_regwrite_out,
               set_memread_out,
               set_memwrite_out,
               clk);

   input [15:0] instruc_in;                // os 16 bits da instrucao
   output reg [3:0] read_reg_out;          // registrador que devera ser lido
   output reg [3:0] write_reg_out;         // registrador que devera ser escrito
   output reg set_regwrite_out;            // indica que havera escrita no registrador
   //output reg [15:0] memwrite_addr_out;  // endereco da memoria onde escreveremos
   output reg set_memwrite_out;            // indica que havera escrita num endereco de memoria
   output reg set_memread_out;             // indica que havera leitura num endereco de memoria
   input clk;
   
   wire [7:0] op;                          // pra deixar o codigo mais compacto
   assign op = instruc_in[15:8];
   
   always @(posedge clk) //(instruc_in, op)
   begin
      if (op == `LW) begin // LW  R, I(R)
         read_reg_out <= instruc_in[3:0];
         write_reg_out <= instruc_in[7:4];
         set_regwrite_out <= 1;
         set_memwrite_out <= 0;
         set_memread_out <= 1;
      end
      else if (op == `LW1) begin // LW  R, I
         read_reg_out <= 4'h0;
         write_reg_out <= instruc_in[7:4];
         set_regwrite_out <= 1;
         set_memwrite_out <= 0;
         set_memread_out <= 0;
      end
      else if (op == `SW) begin // SW  R, I(R)
         read_reg_out <= instruc_in[7:4];
         write_reg_out <= 4'h0;
         set_regwrite_out <= 0;
         set_memwrite_out <= 1;
         set_memread_out <= 0;
      end
      else if (op == `ADD || op == `SUB ||
               op == `MUL || op == `DIV ||
               op == `AND || op == `OR  || op == `CMP )  // operadores aritmeticos/logicos, ex.: ADD R0, R1
      begin
         read_reg_out <= instruc_in[3:0];
         write_reg_out <= instruc_in[7:4];
         set_regwrite_out <= 1;
         set_memwrite_out <= 0;
         set_memread_out <= 0;
      end
      else if (op == `NOT ) begin // NOT  R
         read_reg_out <= instruc_in[7:4];
         write_reg_out <= instruc_in[7:4];
         set_regwrite_out <= 1;
         set_memwrite_out <= 0;
         set_memread_out <= 0;
      end
      else if (op == `JR ) begin // JR R
         read_reg_out <= instruc_in[7:4];
         write_reg_out <= 4'h0;
         set_regwrite_out <= 0;
         set_memwrite_out <= 0;
         set_memread_out <= 0;
      end
      else if (op == `BRFL ) begin // BRFL R, I
         read_reg_out <= instruc_in[3:0];
         write_reg_out <= 4'h0;
         set_regwrite_out <= 0;
         set_memwrite_out <= 0;
         set_memread_out <= 0;
      end
      else if (op == `CALL ) begin // CALL R
         read_reg_out <= instruc_in[7:4];
         write_reg_out <= 4'h0;
         set_regwrite_out <= 0;
         set_memwrite_out <= 0;
         set_memread_out <= 0;
      end
      else if (op == `RET ) begin // RET
         // TODO !
         read_reg_out <= 4'h0;
         write_reg_out <= 4'h0;
         set_regwrite_out <= 0;
         set_memwrite_out <= 0;
         set_memread_out <= 0;
      end
      // e as de Sprites e imgs ainda
      else begin
         write_reg_out <= 4'h0;
         read_reg_out <= 4'h0;
         set_regwrite_out <= 0;
         set_memwrite_out <= 0;
         set_memread_out <= 0;
      end
   end
   
   // ainda rascunho ...
   
endmodule

///////////////////////////////////////////////////////////////////////////////
// Registradores
///////////////////////////////////////////////////////////////////////////////
module registers(sel_reg0,
                 sel_reg1,
                 reg0_out,
                 reg1_out,
                 setwrite_in,
                 sel_regwrite_in,
                 data_in,
                 clk);
                 
   input [3:0] sel_reg0;          // seleciona qual registrador eh primeiro
   input [3:0] sel_reg1;          // seleciona qual registrador eh segundo
   
   output reg [15:0] reg0_out;
   output reg [15:0] reg1_out;
   
   input setwrite_in;             // 1 pra gravar
   input [3:0] sel_regwrite_in;   // seleciona qual registrador a ser escrito
   input [15:0] data_in;          // dado a ser gravado
   input clk;
   
   reg [15:0] regs [0:7];         // TODO: por mais registradores
   
   // demux (escrita)
   always @(posedge clk) begin
      if(setwrite_in == 1) begin
         case(sel_regwrite_in)
            `R0: regs[0] <= data_in;
            `R1: regs[1] <= data_in;
            `R2: regs[2] <= data_in;
            `R3: regs[3] <= data_in;
            default: regs[7] <= data_in;
         endcase
      end
   end
   
   // mux 0 (leitura)
   always @(posedge clk) begin
      case(sel_reg0)
         `ZERO: reg0_out <= 16'h0000;
         `ONE:  reg0_out <= 16'h0001;
         `R0:   reg0_out <= regs[0];
         `R1:   reg0_out <= regs[1];
         `R2:   reg0_out <= regs[2];
         `R3:   reg0_out <= regs[3];
         default: reg0_out <= regs[7];
      endcase
   end
   
   // mux 1 (leitura)
   always @(posedge clk) begin
      case(sel_reg1)
         `ZERO: reg1_out <= 16'h0000;
         `ONE:  reg1_out <= 16'h0001;
         `R0:   reg1_out <= regs[0];
         `R1:   reg1_out <= regs[1];
         `R2:   reg1_out <= regs[2];
         `R3:   reg1_out <= regs[3];
         default: reg1_out <= regs[7];
      endcase
   end

endmodule
   
`endif // _ID_V_
