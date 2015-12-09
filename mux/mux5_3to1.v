module mux5_3to1 (out, in0, in1, in2, sel);
	output reg  [4:0] out;
	input [4:0] in0;
	input [4:0] in1;
	input [4:0] in2;
  	input [1:0] sel;

  	always @(sel)
    begin
		case (sel)
			2'b00: out = in0;
			2'b01: out = in1;
			2'b10: out = in2;
		endcase
    end
endmodule
