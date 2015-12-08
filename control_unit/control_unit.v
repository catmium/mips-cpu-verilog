module control_unit (
	input[5:0] 	op,
	output	reg regdst, regwrite,
	output 	reg branch,
	output	reg jump,
	output	reg memread, memtoreg, memwrite,
	output	reg [1:0] aluop,
	output	reg aluscr );

	always @(*) begin
		// default
		// R-type
		branch 		<= 1'b0;
		jump 		<= 1'b0;
		memread 	<= 1'b0;
		memtoreg	<= 1'b0;
		memwrite	<= 1'b0;
		aluop[1:0]	<= 2'b10;
		aluscr		<= 1'b0;
		regdst 		<= 1'b1;
		regwrite	<= 1'b1;

		case(op)
			6'b10_0011: begin // lw
				regdst		<= 1'b0;
				regwrite	<= 1'b0;
				memread		<= 1'b1;
				aluop[1:0]	<= 2'b00;
				aluscr		<= 1'b1;
			end
			6'b10_1011: begin // sw
				regwrite	<= 1'b0;
				aluop[1:0]	<= 2'b00;
				aluscr		<= 1'b1;
				memwrite 	<= 1'b1;
				regwrite	<= 1'b0;
			end
			6'b00_0100: begin // beq
				branch		<= 1'b1;
				aluop[1:0]	<= 2'b01;
				regwrite	<= 1'b0;
			end
			6'b00_1000: begin // addi
				regdst   	<= 1'b0;
				aluop[1] 	<= 1'b0;
				aluscr   	<= 1'b1;
			end
			6'b00_0000: begin // add
			end
		endcase
	end

endmodule
