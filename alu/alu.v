module alu(
		input		[3:0]	ctl,
		input		[31:0]	a, b,
		output reg	[31:0]	out,
		output				zero);

	wire [31:0] sub_ab;
	wire [31:0] add_ab;
	wire 		slt;

	assign zero = (0 == out);

	assign sub_ab = a - b;
	assign add_ab = a + b;

	assign slt = oflow_sub ? ~(a[31]) : a[31];

	always @(*) begin
		case (ctl)
			4'd0000:  out <= a & b;				/* and */
			4'd0001:  out <= a | b;				/* or */
			4'd0010:  out <= add_ab;				/* add */
			4'd0110:  out <= sub_ab;				/* sub */
			4'd0111:  out <= {{31{1'b0}}, slt};	/* slt */

			default: out <= 0;
		endcase
	end

endmodule