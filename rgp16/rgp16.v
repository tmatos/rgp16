
///////////////////////////////////////////////////////////////////////////////
// Prototipo do processador GRP16
// by Tiago Matos
// Janeiro a Fevereiro de 2017
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Modulo principal
///////////////////////////////////////////////////////////////////////////////
module rgp16( input wire clock,
              //input reset,

               // sinais
               input wire[15:0] newpc,
               input wire[15:0] datain,
               output wire[15:0] DBG_PC,
               output wire[15:0] DBG_INSTRUC,
               output wire[15:0] DBG_IMED,
               output wire[15:0] DBG_A,
               output wire[15:0] DBG_B,
               output wire[15:0] DBG_RESULT,
               output wire[7:0] DBG_ALU_OP,
               output wire[3:0] DBG_DESTINREG,
               output wire DBG_SET_REGWRITE,
               output wire DBG_SET_MEMWRITE,
               input wire sel
            );

   pc if_pc ( .pc_out(w_if_pc_out),
              .new_pc_in(w_if_mux_pc_out),
              .clk(clock) );
   
   pc_inc if_pc_inc ( .pc_in(w_if_pc_out),
                      .pcinc_out(w_if_pc_inc_out) );
   
   mux2pra1 if_mux_pc ( .in0(w_if_pc_inc_out),
                        .in1(newpc),
                        .sel(sel),
                        .out(w_if_mux_pc_out) );
   
   wire [15:0] w_if_pc_out;
   wire [15:0] w_if_pc_inc_out;
   wire [15:0] w_if_mux_pc_out;
   wire [15:0] w_if_memdata_out;
   
   assign DBG_PC = w_if_pc_out;

   wire we;
   assign we = 1'b0; // temporariamente

   ram mem0( .addr_in(w_if_pc_out),
             .data_out(w_if_memdata_out),
             .data_in(datain),
             .write_enable(we),
             .clk(clock) );
             
   wire [15:0] w_id_instruc;
   wire [15:0] w_id_imediat;
   
   assign DBG_INSTRUC = w_id_instruc;

   if_id if_id0 ( .data_in(w_if_memdata_out),
                  .instruc_out(w_id_instruc),
                  .imediat_out(w_id_imediat),
                  .clk(clock) );
   
   wire w_rw_indica;
   assign w_rw_indica = (w_id_instruc[12:8] == 5'b00001);
   
   wire [3:0] w_id_read_reg;
   wire [3:0] w_id_write_reg;
   wire w_id_regwrite;
   wire w_id_memwrite;
   
   control id_control ( .instruc_in(w_id_instruc), 
                        .read_reg_out(w_id_read_reg),
                        .write_reg_out(w_id_write_reg),
                        .set_regwrite_out(w_id_regwrite),
                        .set_memwrite_out(w_id_memwrite) );
   
   wire TMP_rw_in;
   assign TMP_rw_in = 0;
   wire [15:0] TMP_write_in_in;
   assign TMP_write_in_in = 16'h0000;
                   
   wire [15:0] w_id_reg0_out;
   wire [15:0] w_id_reg1_out;

   registers id_regs ( .sel_reg0(w_id_instruc[7:4]),
                       .sel_reg1(w_id_instruc[3:0]),
                       .reg0_out(w_id_reg0_out),
                       .reg1_out(w_id_reg1_out),
                       .setwrite_in(TMP_rw_in),
                       .sel_regwrite_in(4'b0010),
                       .data_in(TMP_write_in_in),
                       .clk(clock) );
   
   id_ex id_ex0 ( .a_in(w_id_reg0_out),
                  .a_out(w_ex_a_reg),
                  .b_in(w_id_reg1_out),
                  .b_out(w_ex_b_reg),
                  .destreg_in(w_id_write_reg),
                  .destreg_out(DBG_DESTINREG),
                  .set_regwrite_in(w_id_regwrite),
                  .set_regwrite_out(DBG_SET_REGWRITE),
                  .set_memwrite_in(w_id_memwrite),
                  .set_memwrite_out(w_ex_memwrite),
                  .opcode_in(w_id_instruc[15:8]),
                  .opcode_out(w_ex_ulactrl_opcode_in),
                  .imedi_in(w_id_imediat),
                  .imedi_out(w_ex_imedi),
                  .tem_bis(w_id_instruc[15]), 
                  .clk(clock) );

endmodule

///////////////////////////////////////////////////////////////////////////////
// if_id interface (ciente dos casos de inst. de 16+16 bits)
///////////////////////////////////////////////////////////////////////////////
module if_id(data_in, instruc_out, imediat_out, clk);
   input [15:0]data_in;
   output reg [15:0] instruc_out;
   output reg [15:0] imediat_out;
   input clk;
   
   reg bis; // indica se estamos lendo ainda a segunda word da instruc.
   
   initial bis = 0;
   
   always @(posedge clk) begin
      if (bis == 0) begin
         instruc_out <= data_in;
         imediat_out <= 16'h0000;
         bis <= data_in[15];
      end
      else if (bis == 1) begin
         imediat_out <= data_in;
         bis <= 0;
      end
   end
endmodule

///////////////////////////////////////////////////////////////////////////////
// id_ex interface
///////////////////////////////////////////////////////////////////////////////
module id_ex(a_in, a_out,
             b_in, b_out,
             destreg_in, destreg_out,
             set_regwrite_in, set_regwrite_out,
             set_memwrite_in, set_memwrite_out,
             opcode_in, opcode_out,
             imedi_in, imedi_out,
             tem_bis, clk
            );
   input [15:0] a_in;
   output reg [15:0] a_out;
   input [15:0] b_in;
   output reg [15:0] b_out;
   //input [4:0] aluop_in;
   //output reg [4:0] aluop_out;
   input [3:0] destreg_in;
   output reg [3:0] destreg_out;
   input set_regwrite_in;
   output reg set_regwrite_out;
   input set_memwrite_in;
   output reg set_memwrite_out;
   input [7:0] opcode_in;
   output reg [7:0] opcode_out;
   input [15:0] imedi_in;
   output reg [15:0] imedi_out;
   input tem_bis;
   input clk;
   
   reg bis; // indica se estamos lendo ainda a segunda word da instruc.
   
   initial bis = 0;
   
   always @(posedge clk) begin
      if (bis == 0) begin
         a_out <= a_in;
         b_out <= b_in;
         //aluop_out <= aluop_in;
         destreg_out <= destreg_in;
         set_regwrite_out <= set_regwrite_in;
         set_memwrite_out <= set_memwrite_in;
         opcode_out <= opcode_in;
         imedi_out <= 16'h0000;
         bis <= tem_bis;
      end
      else if (bis == 1) begin
         a_out <= a_in;
         b_out <= b_in;
         //aluop_out <= aluop_in;
         destreg_out <= destreg_in;
         set_regwrite_out <= set_regwrite_in;
         set_memwrite_out <= set_memwrite_in;
         opcode_out <= opcode_in;
         imedi_out <= imedi_in;
         bis <= 0;
      end
   end
   
endmodule

///////////////////////////////////////////////////////////////////////////////
// PC
///////////////////////////////////////////////////////////////////////////////
module pc(pc_out, new_pc_in, clk);
   output [15:0] pc_out;
   input [15:0] new_pc_in;
   input clk;

   reg[15:0] pc_reg;

   /* por o contador de programa (pc) em zero no inicio */
   initial pc_reg = 0;

   /* atualiza o valor do pc */
   always @(posedge clk) begin
      pc_reg <= new_pc_in;
   end

   assign pc_out = pc_reg;
endmodule

///////////////////////////////////////////////////////////////////////////////
// PC incrementado em 1
///////////////////////////////////////////////////////////////////////////////
module pc_inc(pc_in, pcinc_out);
   input [15:0] pc_in;
   output reg [15:0] pcinc_out;

   always @(pc_in) begin
      pcinc_out <= pc_in + 16'h0001;
   end

endmodule

///////////////////////////////////////////////////////////////////////////////
// mux2pra1 (16 bits wide, usado em varios lugares)
///////////////////////////////////////////////////////////////////////////////
module mux2pra1(in0, in1, sel, out);
   input [15:0] in0;
   input [15:0] in1;
   input sel;
   output reg [15:0] out;

   always @(in0, in1, sel) begin
      if (sel == 0)
         out <= in0;
      else if (sel == 1)
         out <= in1;
      else
         out <= in0;
   end

endmodule

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
      mem[2] <= 16'b0100001100110010 ; // ADD A1, A0
      mem[3] <= 16'b1100001000110000 ; // SW A1, 10(ZERO)
      mem[4] <= 16'b0000000000001101 ; // o 10 acima
      mem[5] <= 16'b1100000100110010 ; // LW A1, 64(A0)
      mem[6] <= 16'b0000000001000000 ; // o 64 acima
      mem[7] <= 16'b0000111100000000 ; // NOP
      //mem[8] <= 
      //mem[9] <= 
      //mem[10] <= 
   end

   always @(posedge clk) begin
      if (write_enable) mem[addr_in] <= data_in;
   end

   always @(posedge clk) begin
      data_out = mem[addr_in];
   end
   
endmodule

///////////////////////////////////////////////////////////////////////////////
// modulo de Controle geral
///////////////////////////////////////////////////////////////////////////////
module control(instruc_in, read_reg_out, write_reg_out, set_regwrite_out,
               /*memwrite_addr_out,*/ set_memwrite_out );
   parameter LW   = 8'b11000001;
   parameter LW1  = 8'b10100001;
   parameter SW   = 8'b11000010;
   parameter ADD  = 8'b01000011;
   parameter SUB  = 8'b01000100;
   parameter MUL  = 8'b01000101;
   parameter DIV  = 8'b01000110;
   parameter AND  = 8'b01000111;
   parameter OR   = 8'b01001000;
   parameter CMP  = 8'b01001001;
   parameter NOT  = 8'b00101010;
   parameter JR   = 8'b00101011;
   parameter BRFL = 8'b10101100;
   parameter CALL = 8'b00101101;
   parameter RET  = 8'b00001110;
   parameter NOP  = 8'b00001111;
   
   //parameter NULO = 16'h0000;

   input [15:0] instruc_in; // os 16 bits da instrucao
   output reg [3:0] read_reg_out; // registrador que devera ser lido
   output reg [3:0] write_reg_out; // registrador que devera ser escrito
   output reg set_regwrite_out; // indica que havera escrita no registrador
   //output reg [15:0] memwrite_addr_out; // endereco da memoria onde escreveremos
   output reg set_memwrite_out; // indica que havera escrita num endereco de memoria
   
   wire [7:0] op; // pra deixar o codigo mais compacto
   assign op = instruc_in[15:8];
   
   always @(instruc_in, op)
   begin
      if (op == LW) begin // LW  R, I(R)
         read_reg_out <= instruc_in[3:0];
         write_reg_out <= instruc_in[7:4];
         set_regwrite_out <= 1;
         //memwrite_addr_out <= NULO;
         set_memwrite_out <= 0;
      end
      else if (op == LW1) begin // LW  R, I
         read_reg_out <= 4'h0;
         write_reg_out <= instruc_in[7:4];
         set_regwrite_out <= 1;
         //memwrite_addr_out <= NULO;
         set_memwrite_out <= 0;
      end
      else if (op == SW) begin // SW  R, I(R)
         read_reg_out <= instruc_in[7:4];
         write_reg_out <= 4'h0;
         set_regwrite_out <= 0;
         //memwrite_addr_out <= NULO;
         set_memwrite_out <= 1;
      end
      else if (op == ADD || op == SUB || op == MUL  || op == DIV ||
               op == AND || op == OR  || op == CMP )  // oper. aritm/logics como: ADD R0, R1
      begin
         read_reg_out <= instruc_in[3:0];
         write_reg_out <= instruc_in[7:4];
         set_regwrite_out <= 1;
         //memwrite_addr_out <= NULO;
         set_memwrite_out <= 0;
      end
      else if (op == NOT ) begin // NOT  R
         read_reg_out <= instruc_in[7:4];
         write_reg_out <= instruc_in[7:4];
         set_regwrite_out <= 1;
         //memwrite_addr_out <= NULO;
         set_memwrite_out <= 0;
      end
      // faltam as Branch controls
      // e as de Sprites e imgs ainda
      else begin
         write_reg_out <= 4'h0;
         read_reg_out <= 4'h0;
         set_regwrite_out <= 0;
         //memwrite_addr_out <= NULO;
         set_memwrite_out <= 0;
      end
   end
   
   // ainda rascunho ...
   
endmodule

///////////////////////////////////////////////////////////////////////////////
// Registradores
///////////////////////////////////////////////////////////////////////////////
module registers(sel_reg0, sel_reg1, reg0_out, reg1_out,
                 setwrite_in, sel_regwrite_in, data_in, clk);
   input [3:0] sel_reg0; // selec qual regist. eh primeiro
   input [3:0] sel_reg1; // selec qual regist. eh segundo
   
   output reg [15:0] reg0_out;
   output reg [15:0] reg1_out;
   
   input setwrite_in; // 1 pra gravar
   input [3:0] sel_regwrite_in; // selec qual regist. a ser escrito
   input [15:0] data_in; // dado a ser gravado
   input clk;
   
   reg [15:0] r0;
   reg [15:0] r1;
   reg [15:0] r2;
   reg [15:0] r3;
   reg [15:0] rLixo;
   
   // demux (escrita)
   always @(posedge clk) begin
      if(setwrite_in == 1) begin
         case(sel_regwrite_in)
            4'b0010: r0 <= data_in;
            4'b0011: r1 <= data_in;
            4'b0100: r2 <= data_in;
            4'b0101: r3 <= data_in;
            default: rLixo <= data_in;
         endcase
      end
   end
   
   // mux 0 (leitura)
   always @(posedge clk) begin
      case(sel_reg0)
         4'b0000: reg0_out <= 16'h0000;
         4'b0001: reg0_out <= 16'h0001;
         4'b0010: reg0_out <= r0;
         4'b0100: reg0_out <= r1;
         4'b0110: reg0_out <= r2;
         4'b1000: reg0_out <= r3;
         default: reg0_out <= rLixo;
      endcase
   end
   
   // mux 1 (leitura)
   always @(posedge clk) begin
      case(sel_reg1)
         4'b0000: reg1_out <= 16'h0000;
         4'b0001: reg1_out <= 16'h0001;
         4'b0010: reg1_out <= r0;
         4'b0100: reg1_out <= r1;
         4'b0110: reg1_out <= r2;
         4'b1000: reg1_out <= r3;
         default: reg1_out <= rLixo;
      endcase
   end

endmodule
