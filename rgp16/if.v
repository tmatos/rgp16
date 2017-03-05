
///////////////////////////////////////////////////////////////////////////////
// instruction fetch - primeira etapa do pipeline
///////////////////////////////////////////////////////////////////////////////

`ifndef _IF_V_

`define _IF_V_

///////////////////////////////////////////////////////////////////////////////
// PC
///////////////////////////////////////////////////////////////////////////////
module pc(pc_out, new_pc_in, clk);
   output [15:0] pc_out;
   input [15:0] new_pc_in;
   input clk;

   reg [15:0] pc_reg;

   // por o contador de programa (pc) em zero no inicio
   initial pc_reg = 16'h0000;

   // atualiza o valor do pc
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
// delay de 1 puslo de clock
///////////////////////////////////////////////////////////////////////////////
module delay(data_in, data_out, clk);
   input [15:0] data_in;
   output reg [15:0] data_out;
   input clk;
   
   always @(posedge clk) begin
      data_out <= data_in;
   end
endmodule

///////////////////////////////////////////////////////////////////////////////
// adaptador para uso de instrucoes variaveis (16 ou 32 bits)
///////////////////////////////////////////////////////////////////////////////
module adap(data_in, instruc_out, imediat_out, clk);
   input [15:0] data_in;
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
   
`endif // _IF_V_
