module alu_control(
		input wire [5:0] funct,
		input wire [1:0] aluop,
		output reg [2:0] aluctl);

	reg [2:0] _funct;

	always @(*) begin
		case(funct[3:0])
			4'b0000:  _funct = 2'b010;	
			4'b0010:  _funct = 2'b110;	
			4'b0100:  _funct = 2'b000;	
			4'b0101:  _funct = 2'b001;	
			4'b1010:  _funct = 2'b111;	
			default: _funct = 2'b0;
		endcase
	end

	always @(*) begin
		case(aluop)
			2'b00: aluctl = 2'b010;
			2'b01: aluctl = 2'b110;	
			2'b10: aluctl = _funct;
			2'b11: aluctl = 2'b110;	
			default: aluctl = 0;
		endcase
	end

endmodule