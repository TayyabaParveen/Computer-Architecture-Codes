module mux32bit (
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
