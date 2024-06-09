
module main_control(
    input [6:0] opcode,
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
//Based on the opcode of the instruction 7 control signals are generated by the control unit
//for different types of instructions as:
  case(opcode)
    7'b0000011: // lw
      begin
      ALUSrc=1;
      MemtoReg=1;
      RegWrite=1;
      MemRead=1;
      MemWrite=0;
      Branch=0;
      ALUOp=00;
      end
    7'b0100011: // sw
      begin
      ALUSrc=1;
      MemtoReg=0;
      RegWrite=0;
      MemRead=0;
      MemWrite=1;
      Branch=0;
      ALUOp=00;
      end
    7'b1100011: // beq
      begin
         ALUSrc=0;
      MemtoReg=0;
      RegWrite=0;
      MemRead=0;
      MemWrite=0;
      Branch=1;
      ALUOp=01;
      end
    7'b0110011: // R-type
      begin
      ALUSrc=0;
      MemtoReg=0;
      RegWrite=1;
      MemRead=0;
      MemWrite=0;
      Branch=0;
      ALUOp=10;
      end
  endcase
end 
endmodule

