module sign_extended (out, in);
  output reg [31:0] out;
  input [15:0] in;

  always@ (in)
    begin
      out={16'b0000_0000_0000_0000,in};
    end
endmodule
