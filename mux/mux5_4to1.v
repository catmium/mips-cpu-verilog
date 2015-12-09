module mux5_4to1 (out, in0, in1, in2, in3, sel);
	output reg  [4:0] out;
	input [4:0] in0;
	input [4:0] in1;
	input [4:0] in2;
	input [4:0] in3;
	input [3:0] sel;

  	always @(sel)
    begin
		case (sel)
			2'b00: out = in0;
			2'b01: out = in1;
			2'b10: out = in2;
			2'b11: out = in3;
		endcase
    end
endmodule
