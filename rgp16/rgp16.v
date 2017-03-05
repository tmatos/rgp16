
///////////////////////////////////////////////////////////////////////////////
// Prototipo do processador GRP16
// by Tiago Matos
// Janeiro a Fevereiro de 2017
///////////////////////////////////////////////////////////////////////////////

`include "if.v"
`include "id.v"
`include "ex.v"
//`include "mem.v"
//`include "wb.v"
`include "constants.v"
`include "ram.v"

///////////////////////////////////////////////////////////////////////////////
// Modulo principal
///////////////////////////////////////////////////////////////////////////////
module grp16( input wire clock,
              //input reset,

               // sinais
               input wire[15:0] newpc,
               output wire[15:0] D_IF_PC,
               output wire[15:0] D_IF_MEMDATA_OUT,
               output wire[15:0] D_ID_INSTRUC,
               output wire[15:0] D_ID_IMEDIAT,
               output wire[15:0] D_EX_IMED,
               output wire[15:0] D_EX_A,
               output wire[15:0] D_EX_B,
               output wire[15:0] D_MEM_RESULT,
               output wire[7:0] D_EX_ALU_OP,
               output wire[3:0] D_MEM_DESTINREG,
               output wire D_MEM_SET_REGWRITE,
               output wire D_MEM_SET_MEMWRITE,
               input wire sel
            );
            
   //== IF stage ============================================================//

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

   assign D_IF_PC = w_if_pc_out;

   ram mem0( .addr_in(w_if_pc_out),
             .data_out(w_if_memdata_out),
             .data_in(16'hFFFF),
             .write_enable(1'b0),
             .clk(clock) );
             
   assign D_IF_MEMDATA_OUT = w_if_memdata_out;

   if_id if_id0 ( .data_in(w_if_memdata_out),
                  .instruc_out(w_id_instruc),
                  .imediat_out(w_id_imediat),
                  .clk(clock) );

   //== ID stage ============================================================//
             
   wire [15:0] w_id_instruc;
   wire [15:0] ct;
   
   wire [15:0] w_id_imediat;
   wire [15:0] w_id_imediat_pos;
   
   delay id_imedi_delay ( .data_in(w_id_imediat),
                          .data_out(w_id_imediat_pos),
                          .clk(clock) );
   
   assign D_ID_INSTRUC = w_id_instruc;
   assign D_ID_IMEDIAT = w_id_imediat;
   
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
                        .set_memwrite_out(w_id_memwrite),
                        .clk(clock) );
   
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
                       .setwrite_in(w_cross_set_regwrite),
                       .sel_regwrite_in(w_cross_write_reg),
                       .data_in(w_cross_data_to_reg),
                       .clk(clock) );
   
   id_ex id_ex0 ( .a_in(w_id_reg0_out),
                  .a_out(w_ex_a_reg),
                  .b_in(w_id_reg1_out),
                  .b_out(w_ex_b_reg),
                  .destreg_in(w_id_write_reg),
                  .destreg_out(w_ex_destin_reg),
                  .set_regwrite_in(w_id_regwrite),
                  .set_regwrite_out(w_ex_regwrite),
                  .set_memwrite_in(w_id_memwrite),
                  .set_memwrite_out(w_ex_memwrite),
                  .opcode_in(w_id_instruc[15:8]),
                  .opcode_out(w_ex_ulactrl_opcode_in),
                  .imedi_in(w_id_imediat_pos),
                  .imedi_out(w_ex_imedi),
                  .clk(clock) );
                  
   //== EX stage ============================================================//
                  
   wire w_ex_regwrite;
   wire w_ex_memwrite;
   wire [3:0] w_ex_destin_reg;
                  
   wire [15:0] w_ex_a_reg;
   wire [15:0] w_ex_b_reg;
   wire [15:0] w_ex_imedi;
  
   wire [7:0] w_ex_ulactrl_opcode_in;
                       
   wire w_ex_amux_sel;
   wire w_ex_bmux_sel;
   wire [3:0] w_ex_ulaop;
  
   ula_control ula_ctr0 ( .opcode_in(w_ex_ulactrl_opcode_in),
                          .ula_amuxsel_out(w_ex_amux_sel),
                          .ula_bmuxsel_out(w_ex_bmux_sel),
                          .ulaop_out(w_ex_ulaop),
                          .clk(clock)
                        );
                    
   wire [15:0] w_ex_ula_aterm;
   wire [15:0] w_ex_ula_bterm;
   
   mux2pra1 mux_a ( .in0(w_ex_a_reg),
                    .in1(16'h0000),
                    .sel(w_ex_amux_sel),
                    .out(w_ex_ula_aterm) );
   
   mux2pra1 mux_b ( .in0(w_ex_b_reg),
                    .in1(w_ex_imedi),
                    .sel(w_ex_bmux_sel),
                    .out(w_ex_ula_bterm) );
   
   ula ula0 ( .a(w_ex_ula_aterm),
              .b(w_ex_ula_bterm),
              .result(w_ex_result),
              /*carryout, over,*/
              .aluop(w_ex_ulaop) );
              
   wire [15:0] w_ex_result;
              
   ex_mem ex_mem0 ( .set_regwrite_in(w_ex_regwrite),
                    .set_regwrite_out(w_cross_set_regwrite),
                    .destreg_in(w_ex_destin_reg),
                    .destreg_out(w_cross_write_reg),
                    .set_memwrite_in(w_ex_memwrite),
                    .set_memwrite_out(D_MEM_SET_MEMWRITE),
                    .memwrite_addr_in(16'h00F0),///////////// !!!!!!!!!!!!
                    .memwrite_addr_out(wb_memwrite_addr),//// !!!!!!!!!!!!
                    .result_in(w_ex_result),
                    .result_out(w_cross_data_to_reg), /// !!!!! temp
                    .clk(clock)
                  );
   
   assign D_EX_A = w_ex_a_reg;
   assign D_EX_B = w_ex_b_reg;
   assign D_EX_IMED = w_ex_imedi;
   assign D_EX_ALU_OP = w_ex_ulaop;


   //== MEM stage ===========================================================//
   
   assign D_MEM_SET_REGWRITE = w_cross_set_regwrite;
   assign D_MEM_DESTINREG = w_cross_write_reg;
   assign D_MEM_RESULT = w_cross_data_to_reg;
   
   wire w_cross_set_regwrite; // liga o flag para escrita do reg de volta nos regs
   wire [3:0] w_cross_write_reg; // liga o indic do reg pra escrita de volta nos regs
   wire [15:0] w_cross_data_to_reg;
                  
   wire wb_memwrite_addr;
   
   // !!!!! TODO: por um mux antes pra esse cara !
   wire [15:0] w_cross_regdata_in; // liga os dados a serem gravados de volta nos regs

endmodule

///////////////////////////////////////////////////////////////////////////////
// if_id interface (ciente dos casos de inst. de 16+16 bits)
///////////////////////////////////////////////////////////////////////////////
module if_id(data_in, instruc_out, imediat_out, clk);
   parameter NOP  = 16'b0000111100000000;

   input [15:0] data_in;
   output reg [15:0] instruc_out;
   output reg [15:0] imediat_out;
   input clk;
   
   reg bis; // indica se estamos lendo ainda a segunda word da instruc.
   reg [15:0] tmp; // guarda temp. uma word da instrucao
   
   initial bis = 1'b0;
   
   always @(posedge clk) begin
      if (bis == 1'b0) begin
         if (data_in[15] == 1'b1) begin // caso instrucao de 32 bits
            instruc_out <= NOP;
            imediat_out <= 16'h0000;
            tmp <= data_in;
            bis <= data_in[15];
         end
         else begin // caso de 16 bits
            instruc_out <= data_in;
            imediat_out <= 16'h0000;
         end
      end
      else if (bis == 1'b1) begin
         instruc_out <= tmp;
         imediat_out <= data_in;
         bis <= 1'b0;
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
             clk
            );
   input [15:0] a_in;
   output reg [15:0] a_out;
   input [15:0] b_in;
   output reg [15:0] b_out;
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
   input clk;
   
   always @(posedge clk) begin
      a_out <= a_in;
      b_out <= b_in;
      destreg_out <= destreg_in;
      set_regwrite_out <= set_regwrite_in;
      set_memwrite_out <= set_memwrite_in;
      opcode_out <= opcode_in;
      imedi_out <= imedi_in;
   end
   
endmodule

///////////////////////////////////////////////////////////////////////////////
// ex_mem interface
///////////////////////////////////////////////////////////////////////////////
module ex_mem( set_regwrite_in, set_regwrite_out,
               destreg_in, destreg_out,
               set_memwrite_in, set_memwrite_out,
               memwrite_addr_in, memwrite_addr_out,
               result_in, result_out, clk
             );
   input set_regwrite_in;
   output reg set_regwrite_out;
   input [3:0] destreg_in;
   output reg [3:0] destreg_out;
   input set_memwrite_in;
   output reg set_memwrite_out;
   input [15:0] memwrite_addr_in;
   output reg [15:0] memwrite_addr_out;
   input [15:0] result_in;
   output reg [15:0] result_out;
   input clk;
   
   always @(posedge clk) begin
      set_regwrite_out <= set_regwrite_in;
      destreg_out <= destreg_in;
      set_memwrite_out <= set_memwrite_in;
      memwrite_addr_out <= memwrite_addr_in;
      result_out <= result_in;
   end
             
endmodule

///////////////////////////////////////////////////////////////////////////////
// mem_wb interface
///////////////////////////////////////////////////////////////////////////////
//module ex_mem(a_in, a_out,
//              b_in, b_out,
//              clk );
//endmodule

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
