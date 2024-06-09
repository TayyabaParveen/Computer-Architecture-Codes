//Inputs and Outputs
module regfile(
input [4:0] read_register_1,
input [4:0] read_register_2,
input RegWrite,
input [4:0] write_register,
input [31:0] write_data,
output reg[31:0] read_data_1,
output reg[31:0] read_data_2
);
reg [31:0] registers[31:0];
//Manually giving 32-bit values for 32 register
initial begin
registers[0] = 32'b00000000000000000000000000000000;
registers[1] = 32'b00000000000000000000000000000001;
registers[2] = 32'b00000000000000000000000000000010;
registers[3] = 32'b00000000000000000000000000000011;
registers[4] = 32'b00000000000000000000000000000100;
registers[5] = 32'b00000000000000000000000000000101;
registers[6] = 32'b00000000000000000000000000000110;
registers[7] = 32'b00000000000000000000000000000111;
registers[8] = 32'b00000000000000000000000000001000;
registers[9] = 32'b00000000000000000000000000001001;
registers[10] = 32'b00000000000000000000000000001010;
registers[11] = 32'b00000000000000000000000000001011;
registers[12] = 32'b00000000000000000000000000001100;
registers[13] = 32'b00000000000000000000000000001101;
registers[14] = 32'b00000000000000000000000000001110;
registers[15] = 32'b00000000000000000000000000001111;
registers[16] = 32'b00000000000000000000000000010000;
registers[17] = 32'b00000000000000000000000000010001;
registers[18] = 32'b00000000000000000000000000010010;
registers[19] = 32'b00000000000000000000000000010011;
registers[20] = 32'b00000000000000000000000000010100;
registers[21] = 32'b00000000000000000000000000010101;
registers[22] = 32'b00000000000000000000000000010110;
registers[23] = 32'b00000000000000000000000000010111;
registers[24] = 32'b00000000000000000000000000011000;
registers[25] = 32'b00000000000000000000000000011001;
registers[26] = 32'b00000000000000000000000000011010;
registers[27] = 32'b00000000000000000000000000011011;
registers[28] = 32'b00000000000000000000000000011100;
registers[29] = 32'b00000000000000000000000000011101;
registers[30] = 32'b00000000000000000000000000011110;
registers[31] = 32'b00000000000000000000000000011111;
end
//o/p 1 and 2 shows data from i/p 1 and 2 based on registers index specified respectively
always @* begin
read_data_1 = registers[read_register_1];
read_data_2 = registers[read_register_2];
// Update registers when RegWrite is 1
if (RegWrite) begin
registers[write_register] <= write_data;
end
end
endmodule