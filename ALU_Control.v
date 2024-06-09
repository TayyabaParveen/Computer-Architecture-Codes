module alu_control(
    input [5:0] alu_control_input,
    output reg [3:0] control_signal
);

always @(*)
begin
    case(alu_control_input)
        // First two bits are ALUOp; next four bits are fun[30,14,13,12] for corresponding control signals
        6'b000000: // lw, sw
            control_signal = 4'b0010;
        6'b010000: // beq
            control_signal = 4'b0110;
        6'b100000: // add
            control_signal = 4'b0010;
        6'b101000: // sub
            control_signal = 4'b0110;
        6'b100111: // and
            control_signal = 4'b0000;
        6'b100110: // or
            control_signal = 4'b0001;
    endcase
end

endmodule
