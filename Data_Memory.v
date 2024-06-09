module DataMemory (Address, Writedata, Control, MemRead, MemWrite, ReadData);
input [31:0] Address;
input [31:0] Writedata;
input Control;
input MemRead, MemWrite;
output reg [31:0] ReadData;
reg [31:0] Mem[5:0];
initial begin
//putting some initial binary values in memory to be read or when written on these same
//addresses modified
Mem[0] = 32'b00000000000000000000000000000000;
Mem[1] = 32'b00000000000000000000000000000001;
Mem[2] = 32'b00000000000000000000000000000010;
Mem[3] = 32'b00000000000000000000000000000011;
Mem[4] = 32'b00000000000000000000000000000100;
Mem[5] = 32'b00000000000000000000000000000101;
end
always @* begin
//when control signal is one means data memory will be used
if (Control) begin
//when MemRead is enabled based on specific address 32 bit binary is read from memory and
//given to ReadData pin
if (MemRead) begin
ReadData = Mem[Address];
//When MemWrite is enabled based on address specified 32 bit binary is written on
//that address in memory
end else if (MemWrite) begin
Mem[Address] <= Writedata;
end
end else begin // Control signal is 0, hold output to 0
ReadData = 32'b0;
end
end
endmodule
