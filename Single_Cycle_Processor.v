module InstructionMemory (address, instruction);
input [31:0] address;
output reg [31:0] instruction;
always @(*) begin
case (address)
32'd80000: instruction = 32'b00000001000111010000010100110011;//add
32'd80004: instruction = 32'b00000001010001010011010010000011;//sub
32'd80008: instruction = 32'b00000000100011000000011010100001;//and
32'd80012: instruction = 32'b0000000_00101_00101_111_01001_0101010;//beq

32'd80016: instruction = 32'b00000000111111000100010010000000;//or
32'd80020: instruction = 32'b000000000011_11010_100_01001_0010100;//lw
32'd80024: instruction = 32'b0000000_00001_00100_110_11010_1010101;//sw


endcase
end
endmodule




//------------------------------------------------------------------------------------------------------------------//

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
if (RegWrite==1) begin
registers[write_register] <= write_data;
end
end
endmodule

//------------------------------------------------------------------------------------------------------------------//

module immed_generator(
    input [31:0] instruction ,
    output reg [31:0] out_immediate
);
 

    always @(*)
    begin
        case(instruction[6:0])
//takes immediate bits(12) from 32 bit instructions and pads with 0s(32 bits)
           
            7'b0110011: // add
                out_immediate = 32'b00000000000000000000000000000000;
            7'b0000011: // sub
                out_immediate = 32'b00000000000000000000000000000000;
            7'b0100001: // and
                out_immediate = 32'b00000000000000000000000000000000;
              7'b0000000: // or
                out_immediate = 32'b00000000000000000000000000000000;  
                7'b0010100://lw
                   out_immediate = {20'b00000000000000000000, instruction[31:20]};
               7'b1010101://sw
                   out_immediate = {20'b00000000000000000000,instruction[31:25], instruction[11:7]};        

          7'b0101010://beq
                   out_immediate = {20'b00000000000000000000,instruction[31], instruction[29:25], instruction[11], instruction[10:7], instruction[30]};    
        endcase
    end

endmodule


//------------------------------------------------------------------------------------------------------------------//
// ALU MUX

module mux2 (
input [31:0] in0, // First 32-bit input
input [31:0] in1, // Second 32-bit input
input select, // Selection line
output reg [31:0] out // 32-bit output
);
always @(*) begin
if (select) begin
out = in1; // Select in1 if select is high
end else begin
out = in0; // Select in0 if select is low
end
end
endmodule



//------------------------------------------------------------------------------------------------------------------//


module ALU_CONTROL(
 input [1:0] ALU0p,
 input [31:0] instruction,//input
 output reg [3:0] ALU_CONTROL //output
 );
 
 always @*
 begin
 // R-Type instruction
 if ({ALU0p[1:0] ,instruction[30] , instruction[14:12]} == 6'b100000 ) begin
 ALU_CONTROL = 4'b0000 ;
 end
 else if ({ALU0p[1:0] ,instruction[30] , instruction[14:12]} == 6'b010011 ) begin
 ALU_CONTROL = 4'b0011 ;
 end
  else if ({ALU0p[1:0] ,instruction[30] , instruction[14:12]} == 6'b110000 ) begin
 ALU_CONTROL = 4'b0111 ;
 end
  else if ({ALU0p[1:0] ,instruction[30] , instruction[14:12]} == 6'b000100 ) begin
 ALU_CONTROL = 4'b1111 ;
 end
else if ({ALU0p[1:0] ,instruction[30] , instruction[14:12]} == 6'b010100 ) begin
 ALU_CONTROL = 4'b0000 ;
 end
else if ({ALU0p[1:0] ,instruction[30] , instruction[14:12]} == 6'b000110) begin
 ALU_CONTROL = 4'b0000 ;
 end
else if ({ALU0p[1:0] ,instruction[30] , instruction[14:12]} == 6'b000111) begin
 ALU_CONTROL = 4'b0011 ;
 end
 else begin
    ALU_CONTROL = 4'b0001 ;
    end
 end
endmodule

//------------------------------------------------------------------------------------------------------------------//
module ALU(
output reg [31:0] O,
output zeroFlag,
input [31:0] A,
input [31:0] B,
input [3:0] S
);
//Inputs and Output are 32 bits each and selection line is 4 bits while zero flag is 1 bit
always @*
begin
if (S == 4'b0000) begin
  O = A+B;
end
else if (S == 4'b0011) begin
  O = A-B;
end
else if (S == 4'b0111) begin
  O = A&B;
end
else if (S == 4'b1111) begin
  O = A|B;
end

else begin
  O = 32'b0 ;
end


end
assign zeroFlag = (O == 32'b0) ? 1'b1 : 1'b0;
//If output is all 32 bits 0s, zero flag is assigned value 1(1 bit) otherwise 0(1 bit)
endmodule



//------------------------------------------------------------------------------------------------------------------//

module DataMemory (Address, Writedata, MemRead, MemWrite, ReadData);
input [31:0] Address;
input [31:0] Writedata;
input MemRead, MemWrite;
output reg [31:0] ReadData;
reg [31:0] Mem[31:0];
initial begin
//putting some initial binary values in memory to be read or when written on these same
//addresses modified
Mem[0] = 32'b00000000000000000000000000000000;
Mem[1] = 32'b00000000000000000000000000000001;
Mem[2] = 32'b00000000000000000000000000000010;
Mem[3] = 32'b00000000000000000000000000000011;
Mem[4] = 32'b00000000000000000000000000000100;
Mem[5] = 32'b00000000000000000000000000000101;
Mem[6] = 32'b00000000000000000000000000000110;
Mem[7] = 32'b00000000000000000000000000000111;
Mem[8] = 32'b00000000000000000000000000001000;
Mem[9] = 32'b00000000000000000000000000001001;
Mem[10] = 32'b00000000000000000000000000001010;
Mem[11] = 32'b00000000000000000000000000001011;
Mem[12] = 32'b00000000000000000000000000001100;
Mem[13] = 32'b00000000000000000000000000001101;
Mem[14] = 32'b00000000000000000000000000001110;
Mem[15] = 32'b00000000000000000000000000001111;
Mem[16] = 32'b00000000000000000000000000010000;
Mem[17] = 32'b00000000000000000000000000010001;
Mem[18] = 32'b00000000000000000000000000010010;
Mem[19] = 32'b00000000000000000000000000010011;
Mem[20] = 32'b00000000000000000000000000010100;
Mem[21] = 32'b00000000000000000000000000010101;
Mem[22] = 32'b00000000000000000000000000010110;
Mem[23] = 32'b00000000000000000000000000010111;
Mem[24] = 32'b00000000000000000000000000011000;
Mem[25] = 32'b00000000000000000000000000011001;
Mem[26] = 32'b00000000000000000000000000011010;
Mem[27] = 32'b00000000000000000000000000011011;
Mem[28] = 32'b00000000000000011000000000011100;
Mem[29] = 32'b00000000000000000000001100011101;
Mem[30] = 32'b00000000000000000001100000011110;
Mem[31] = 32'b00000000000000000000000000011111;
end
always @* begin
//when control signal is one means data memory will be used
if (MemRead) begin
ReadData = Mem[Address];
//When MemWrite is enabled based on address specified 32 bit binary is written on
//that address in memory
end else if (MemWrite) begin
Mem[Address] <= Writedata;
end else begin // Control signal is 0, hold output to 0
ReadData = 32'b0;
end
end
endmodule

//------------------------------------------------------------------------------------------------------------------//
//datamem mux
module mux3 (
input [31:0] data, // First 32-bit input
input [31:0] aluout, // Second 32-bit input
input S, // Selection line
output reg [31:0] Y // 32-bit output
);
always @(*) begin
if (S) begin
Y = data; // Select inB if select is high
end else begin
Y = aluout; // Select inA if select is low
end
end
endmodule
//------------------------------------------------------------------------------------------------------------------//
module and_gate (
  input  A,
  input  B,
  output reg O
);

  always @* begin
    O = A & B;
  end

endmodule
//------------------------------------------------------------------------------------------------------------------//

module main_control(
    input [6:0] instruction,
    output reg Branch,
    output reg MemRead,
    output reg MemtoReg,
    output reg [1:0] ALUOp,
    output reg MemWrite,
    output reg ALUSrc,
    output reg RegWrite
);

always @(*)
begin
    // Based on the opcode of the instruction, 7 control signals are generated by the control unit
    // for different types of instructions
    case(instruction)

        7'b0110011: // R-type add
            begin
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp = 2'b10;
            end

        7'b0000011: // sub
            begin
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp = 2'b01;
            end

        7'b0100001: // and
            begin
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp = 2'b11;
            end

        7'b0000000: // or
            begin
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOp = 2'b00;
            end
  7'b0010100: // lw
            begin
                ALUSrc = 1;
                MemtoReg = 1;
                RegWrite = 1;
                MemRead = 1;
                MemWrite = 0;
                Branch = 0;
                ALUOp = 2'b01;
            end
  7'b1010101: // sw
            begin
                ALUSrc = 1;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 1;
                Branch = 0;
                ALUOp = 2'b00;
            end
7'b0101010: // beq
            begin
                ALUSrc = 0;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                Branch = 1;
                ALUOp = 2'b00;
            end

    endcase
end

endmodule

//------------------------------------------------------------------------------------------------------------------//
module processor (
  input [31:0] adress,
  output [31:0] instr,
  output [31:0] read_data1 ,
  output [31:0] read_data2 ,
  output [31:0] write_mux_data,
  output [31:0] offset_immidiate,
  output [31:0] ALU_result,
  output zero_flag,
  output sel,
  output [31:0] read_dm,
  output [3:0] Alu_control_signal
 
  );
   
  InstructionMemory IM(
      .address(adress),
      .instruction(instr)
      );
     
      wire [4:0] readreg1 ;
      assign  readreg1 = instr[19:15] ;
      wire [4:0] readreg2 ;
      assign  readreg2 = instr[24:20] ;
      wire [4:0] writereg ;
      assign  writereg = instr[11:7] ;
      wire [6:0] controlbits ;
      assign  controlbits = instr[6:0] ;  
      wire RegisterWrite ;
      //wire [31:0] write_mux_data ;
     
  regfile RF(
      .read_register_1(readreg1),
      .read_register_2(readreg2),
      .RegWrite(RegisterWrite),
      .write_register(writereg),
      .write_data(write_mux_data),
      .read_data_1(read_data1),
      .read_data_2(read_data2)
);
 
 
 immed_generator IMG(
      .instruction(instr) ,
      .out_immediate(offset_immidiate)
);

 wire ALU_src ;
 wire [31:0] Alu_in_mux_out ;
 
 mux2 Alu_MUX(
     .in0(read_data2), // First 32-bit input
     .in1(offset_immidiate), // Second 32-bit input
     .select(ALU_src), // Selection line
     .out(Alu_in_mux_out) // 32-bit output
);

wire [1:0] ALU_op ;


ALU_CONTROL alu_control(
       .ALU0p(ALU_op),
       .instruction(instr),//input
       .ALU_CONTROL(Alu_control_signal) //output
 );
 
 
 ALU  Main_Alu(
        .O(ALU_result),
        .zeroFlag(zero_flag),
        .A(read_data1),
        .B(Alu_in_mux_out),
        .S(Alu_control_signal)
        );
   
        wire Control_mem_read ;
        wire Control_mem_write ;
DataMemory D_M(
          .Address(ALU_result),
          .Writedata(read_data2),
          .MemRead(Control_mem_read),
          .MemWrite(Control_mem_write),
          .ReadData(read_dm)
          );
         
       wire Control_mem_to_reg ;
       
mux3 D_M_Mux(
         .data(read_dm), // First 32-bit input
         .aluout(ALU_result), // Second 32-bit input
         .S(Control_mem_to_reg), // Selection line
         .Y(write_mux_data) // 32-bit output
);

 wire Control_Branch ;
and_gate and_gate_zero_flag(
        .A(Control_Branch),
        .B(zero_flag),
        .O(sel)
);


           
wire [6:0] w;
assign w = instr[6:0];
main_control  main_control_unit(
    .instruction(w),
    .Branch(Control_Branch),
    .MemRead(Control_mem_read),
    .MemtoReg(Control_mem_to_reg),
    .ALUOp(ALU_op),
    .MemWrite(Control_mem_write),
    .ALUSrc(ALU_src),
    .RegWrite(RegisterWrite)
);
endmodule 

