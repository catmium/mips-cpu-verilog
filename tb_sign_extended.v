// Sign_extended module testbench 
// for MIPS CPU Project
module tb_sign_extended;
  reg [15:0] in;
  wire [31:0] out; 
  
  sign_extended SE1(in, out);
  initial
    begin
      $display("RESULT");
      $monitor("A = %b B = %b",in,out);
      #10 in=16'b1111111111111111;  
      #10 in=16'b0000000000000000;
      #10 in=16'b0101010101010101;
    end
endmodule