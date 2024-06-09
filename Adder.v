//Inputs and outputs
module adder(output [31:0] O,
output zeroFlag,
output Cout,
input [31:0] A,
input Cin,
input [31:0] B);
//adding both i/ps and Carry in
assign O = A + B + Cin;
//to check for carry out
assign Cout = A[31] & B[31];
//to check for zero flag
assign zeroFlag = (O == 32'b0) ? 1'b1 : 1'b0;
endmodule
