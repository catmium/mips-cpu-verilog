module instr_mem (	input [31:0] address,
					output [31:0] instruction );

 	reg [31:0] memory [249:0];
 	integer i;

	initial
		begin
			for (i=0; i<250; i=i+1) memory[i] = 32'b0;
			// Insert MIPS's assembly here start at memory[10] = ...
			memory[10] = 32'b001000_00000_01000_0000000000000010; //addi $t0, $0, 2
			memory[11] = 32'b101011_00100_00101_00000_00000_000000;    //110100 sw
            memory[12] = 32'b100011_00100_00110_00000_00000_000000; 	// 111000 lw
		end

	assign instruction = memory[address >> 2];

endmodule
