module InstructionMemory (address, instruction);
input [31:0] address;

//32 bit address which will be written in binary for any specific instruction

output reg [31:0] instruction;
always @(*) begin
case (address)
//using case statement 32 bit instruction is fetched and shown on o/p based on
//address written in binary
32'b00000000000000010011100010000000: instruction =
32'b00000000001010110001010100010011;
32'b00000000000000010011100010000100: instruction =
32'b00000001100101010000010100110011;
32'b00000000000000010011100010001000: instruction =
32'b00000000000001010011010010000011;
32'b00000000000000010011100010001100: instruction =
32'b00000001100001001001011001100011;
32'b00000000000000010011100010010000: instruction =
32'b00000000000110110000101100010011;
32'b00000000000000010011100010010100: instruction =
32'b11111110000000000000011011100011;
//if address specified is not mentioned then it gives 32 bit binary
//instruction as default
default: instruction = 32'b0;
endcase
end
endmodule
