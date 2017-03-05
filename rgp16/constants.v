
///////////////////////////////////////////////////////////////////////////////
// arquivo cabecalho para os parameters que definem os
// codigos binarios dos opcodes e dos registradores
///////////////////////////////////////////////////////////////////////////////

`ifndef _CONSTANTS_V_

`define _CONSTANTS_V_

// opcodes
`define  LW   8'b11000001
`define  LW1  8'b10100001
`define  SW   8'b11000010
`define  ADD  8'b01000011
`define  SUB  8'b01000100
`define  MUL  8'b01000101
`define  DIV  8'b01000110
`define  AND  8'b01000111
`define  OR   8'b01001000
`define  CMP  8'b01001001
`define  NOT  8'b00101010

// registradores
`define  ZERO 4'b0000
`define  ONE  4'b0001
`define  R0   4'b0010
`define  R1   4'b0011
`define  R2   4'b0100
`define  R3   4'b0101

// como o NULL do C
`define  NULO 16'h0000

`endif // _CONSTANTS_V_
