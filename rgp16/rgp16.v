
module rgp16(in, out, reset, clk);

input reset;
input clk;

input [15:0] in;
output [15:0] out;

reg [15:0] pc;

always @ (posedge clk )//or posedge reset)
begin
   if (reset == 1'b1) begin
      //pc <= '0;
      pc <= in;
   end
   else begin
      pc <= pc + 1;
   end
end

assign out = pc;

endmodule
