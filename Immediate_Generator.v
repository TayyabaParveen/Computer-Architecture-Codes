module immed_generator(
    input [31:0] input_instruction,
    output reg [31:0] processed_instruction
);

    reg [31:0] instruction[5:0];

    initial begin
        instruction[0] = 32'b00000000001010110001010100010011; // slli
        instruction[1] = 32'b00000001100101010000010100110011; // add
        instruction[2] = 32'b00000000000001010011010010000011; // lw
        instruction[3] = 32'b00000001100001001001011001100011; // bne
        instruction[4] = 32'b00000000000110110000101100010011; // addi
        instruction[5] = 32'b11111110000000000000011011100011; // beq
    end

    always @(*)
    begin
        case(input_instruction[6:0])
//takes immediate bits(12) from 32 bit instructions and pads with 0s(32 bits)
            7'b0010011: // slli
                processed_instruction = {20'b000000000000, input_instruction[31:20]};
            7'b0000011: // lw
                processed_instruction = {20'b000000000000, input_instruction[31:20]};
            7'b1100011: // bne
                processed_instruction = {20'b000000000000, input_instruction[31:25], input_instruction[11:7]};
            7'b0010011: // addi
                processed_instruction = {20'b000000000000, input_instruction[31:20]};
            7'b1100011: // beq
                processed_instruction = {20'b000000000000, input_instruction[31:25], input_instruction[11:7]};
            default: // default case
//in case of add instruction as immediate gen is not required so for this code only I am using this condition otherwise for add both i/ps of alu are selected from
//register file not immediate generator
                processed_instruction = 32'b00000000000000000000000000000000;
        endcase
    end

endmodule


