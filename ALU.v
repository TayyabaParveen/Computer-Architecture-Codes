module thirtytwo_Bit_ALU(output [31:0] O,
output zeroFlag,
input [31:0] A,
input [31:0] B,
input [3:0] S);
//Inputs and Output are 32 bits each and selection line is 4 bits while zero flag is 1 bit
wire [31:0] a, b, c, d;
assign a = A & B;
assign b = A | B;
assign c = A + B;
assign d = A - B;
assign O = (S == 4'b0000) ? a :
(S == 4'b0001) ? b :
(S == 4'b0010) ? c :
(S == 4'b0011) ? d : 32'b0;
//Checks if selection bit is 0000 it performs
// AND operation, if selection bit is 0001 it performs OR operation,
//if selection bit is 0010 it performs ADD operation and lastly
//if selection bit is 0011 it performs Subtraction operation otherwise
//it assigns 32 bit 0s as default value.
assign zeroFlag = (O == 32'b0) ? 1'b1 : 1'b0;
//If output is all 32 bits 0s, zero flag is assigned value 1(1 bit) otherwise 0(1 bit)
endmodule
