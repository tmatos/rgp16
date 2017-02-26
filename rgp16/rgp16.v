
///////////////////////////////////////////////////////////////////////////////
// Prototipo do processador GRP16
// by Tiago Matos

///////////////////////////////////////////////////////////////////////////////
// Modulo principal
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
               output wire[7:0] DBG_OPCODE,
               output wire[3:0] DBG_DESTINREG,
               output wire DBG_SET_REGWRITE,
               input wire sel
            );

   pc pc0 ( .pc_out(w_pc_out),
            .new_pc(w_mux_pc),
            .clk(clock) );
   
   pc_inc pc_inc0 ( .pc_in(w_pc_out),
                    .pc_out(w_pc_inc) );
   
   mux_pc mux_pc0 ( .val_inc(w_pc_inc),
                    .val_jump(newpc),
                    .sel(sel),
                    .out_pc(w_mux_pc) );
   
   wire [15:0] w_pc_out;
   wire [15:0] w_pc_inc;
   wire [15:0] w_mux_pc;
   wire [15:0] w_inst_da_mem;
   
   assign DBG_PC = w_pc_out;

   wire we;
   assign we = 1'b0; // temporariamente

   ram mem0( .addr(w_pc_out),
             .data_out(w_inst_da_mem),
             .data_in(datain),
             .write_enable(we),
             .clk(clock) );
             
   wire [15:0] w_instruc;
   wire [15:0] w_imedi;
   
   assign DBG_INSTRUC = w_instruc;

   if_id if_id0 ( .in(w_inst_da_mem),
                  .out(w_instruc),
                  .imedi(w_imedi),
                  .clk(clock) );
   
   wire w_rw_indica;
   assign w_rw_indica = (w_instruc[12:8] == 5'b00001);
   
   wire [3:0] w_read_reg;
   wire [3:0] w_write_reg;
   wire w_regwrite;
   
   control ctrl0 ( .instrucao(w_instruc), 
                   .out_read_reg(w_read_reg),
                   .out_write_reg(w_write_reg),
                   .set_regwrite_out(w_regwrite) );
   
   wire TMP_rw_in;
   assign TMP_rw_in = 0;
   wire [15:0] TMP_write_in_in;
   assign TMP_write_in_in = 16'h0000;
                   
   wire [15:0] w_reg0;
   wire [15:0] w_reg1;

   registers reg0 ( .sel_reg0(w_instruc[7:4]),
                    .sel_reg1(w_instruc[3:0]),
                    .reg0_out(w_reg0),
                    .reg1_out(w_reg1),
                    .rw(TMP_rw_in),
                    .write_in(TMP_write_in_in),
                    .clk(clock) );
   
   id_ex id_ex0 ( .a_in(w_reg0),
                  .a_out(DBG_A), 
                  .b_in(w_reg1), 
                  .b_out(DBG_B), 
                  //.aluop_in(),
                  //.aluop_out(), 
                  .destreg_in(w_write_reg),
                  .destreg_out(DBG_DESTINREG),
                  .set_regwrite_in(w_regwrite),
                  .set_regwrite_out(DBG_SET_REGWRITE),
                  .opcode_in(w_instruc[15:8]),
                  .opcode_out(DBG_OPCODE),
                  .imedi_in(w_imedi),
                  .imedi_out(DBG_IMED),
                  .tem_bis(w_instruc[15]), 
                  .clk(clock) );

endmodule

///////////////////////////////////////////////////////////////////////////////
// if_id interface (ciente dos casos de inst. de 16+16 bits)
module if_id(in, out, imedi, clk);
   input [15:0] in;
   output reg [15:0] out;
   output reg [15:0] imedi;
   input clk;
   
   reg bis; // indica se estamos lendo ainda a segunda word da instruc.
   
   initial bis = 0;
   
   always @(posedge clk) begin
      if (bis == 0) begin
         out <= in;
         imedi <= 16'h0000;
         bis <= in[15];
      end
      else if (bis == 1) begin
         imedi <= in;
         bis <= 0;
      end
   end
endmodule

///////////////////////////////////////////////////////////////////////////////
// id_ex interface
module id_ex(a_in, a_out, b_in, b_out, //aluop_in, aluop_out,
             destreg_in, destreg_out, set_regwrite_in, set_regwrite_out,
             opcode_in, opcode_out, imedi_in, imedi_out, tem_bis, clk);
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
         opcode_out <= opcode_in;
         imedi_out <= imedi_in;
         bis <= 0;
      end
   end
   
endmodule

///////////////////////////////////////////////////////////////////////////////
// PC
module pc(pc_out, new_pc, clk);
   output [15:0] pc_out;
   input [15:0] new_pc;
   input clk;

   reg[15:0] pc_reg;

   /* por o contador de programa (pc) em zero no inicio */
   initial pc_reg = 0;

   /* atualiza o valor do pc */
   always @(posedge clk) begin
      pc_reg <= new_pc;
   end

   assign pc_out = pc_reg;
endmodule

///////////////////////////////////////////////////////////////////////////////
// incrementa PC
module pc_inc(pc_in, pc_out);
   input [15:0] pc_in;
   output reg [15:0] pc_out;

   always @(pc_in) begin
      pc_out <= pc_in + 16'h0001;
   end

endmodule

///////////////////////////////////////////////////////////////////////////////
// novo PC mux
module mux_pc(val_inc, val_jump, sel, out_pc);
   input [15:0] val_inc;
   input [15:0] val_jump;
   input sel;
   output reg [15:0] out_pc;

   always @(val_inc, val_jump, sel) begin
      if (sel == 0)
         out_pc <= val_inc;
      else if (sel == 1)
         out_pc <= val_jump;
      else
         out_pc <= val_inc;
   end

endmodule

///////////////////////////////////////////////////////////////////////////////
// RAM
module ram(
    output reg [15:0] data_out,
    input wire [15:0] addr,
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
      mem[2] <= 16'b0100001100110001 ; // ADD A1, A0
      mem[3] <= 16'b1100000100110001 ; // LW A1, 64(A0)
      mem[4] <= 16'b0000000001000000 ; // o 64 acima
      mem[5] <= 16'b0000111100000000 ; // NOP
   end

    always @(posedge clk)
    begin
        if (write_enable)
        begin
            mem[addr] <= data_in;
        end
    end

	always @ (addr)
	begin
		data_out = mem[addr];
    end

endmodule

///////////////////////////////////////////////////////////////////////////////
// modulo de Controle geral
module control(instrucao, out_read_reg, out_write_reg, set_regwrite_out);
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

   input [15:0] instrucao; // os 16 bits da instrucao
   output reg [3:0] out_read_reg; // registrador que devera ser lido
   output reg [3:0] out_write_reg; // registrador que devera ser escrito
   output reg set_regwrite_out; // indica que havera escrita no registrador
   
   wire [7:0] op; // pra deixar mais o codigo compacto
   assign op = instrucao[15:8];
   
   always @(instrucao)
   begin
      if (op == LW) begin // LW  R, I(R)
         out_read_reg <= instrucao[3:0];
         out_write_reg <= instrucao[7:4];
         set_regwrite_out <= 1;
      end
      else if (op == LW1) begin // LW  R, I
         out_read_reg <= 4'h0;
         out_write_reg <= instrucao[7:4];
         set_regwrite_out <= 1;
      end
      else if (op == SW) begin // SW  R, I(R)
         out_read_reg <= instrucao[7:4];
         out_write_reg <= 4'h0;
         set_regwrite_out <= 0;
      end
      else if (op == ADD || op == SUB || op == MUL  || op == DIV ||
               op == AND || op == OR  || op == CMP )  // oper. aritm/logics como: ADD R0, R1
      begin
         out_read_reg <= instrucao[3:0];
         out_write_reg <= instrucao[7:4];
         set_regwrite_out <= 1;
      end
      else if (op == NOT ) begin // NOT  R
         out_read_reg <= instrucao[7:4];
         out_write_reg <= instrucao[7:4];
         set_regwrite_out <= 1;
      end
      // faltam as Branch controls
      // e as de Sprites e imgs ainda
      else begin
         out_write_reg <= 4'h0;
         out_read_reg <= 4'h0;
         set_regwrite_out <= 0;
      end
   end
   
   // ainda rascunho ...
   
endmodule

///////////////////////////////////////////////////////////////////////////////
// Registradores
module registers(sel_reg0, sel_reg1, reg0_out, reg1_out, rw, write_in, clk);
   input [3:0] sel_reg0; // selec qual regist. eh primeiro
   input [3:0] sel_reg1; // selec qual regist. eh segundo
   
   output reg [15:0] reg0_out;
   output reg [15:0] reg1_out;
   
   input rw; // 0 pra ler, 1 pra gravar
   input [15:0] write_in;
   input clk;
   
   reg [15:0] r0;
   reg [15:0] r1;
   reg [15:0] r2;
   reg [15:0] r3;
   reg [15:0] rLixo;
   
   // demux (escrita)
   always @(posedge clk) begin
      if(rw == 1) begin
         case(sel_reg0)
            4'b0010: r0 <= write_in;
            4'b0011: r1 <= write_in;
            4'b0100: r2 <= write_in;
            4'b0101: r3 <= write_in;
            default: rLixo <= write_in;
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
