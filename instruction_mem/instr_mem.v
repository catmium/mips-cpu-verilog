module instr_mem (	input [31:0] address,
					output [31:0] instruction );

 	reg [31:0] memory [249:0];
 	integer i;

	initial
		begin
			for (i=0; i<250; i=i+1) memory[i] = 32'b0;
			// Insert MIPS's assembly here start at memory[10] = ...
			memory[10] = 32'b001000_00000_01000_00000_00000_000101;    // addi $t0, $0, 5
		end

	assign instruction = memory[address >> 2];

endmodule
