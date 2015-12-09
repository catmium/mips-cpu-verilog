module datamem (
  input clk,
  output [31:0] dataOut,
  input [31:0] address, dataIn,
  input writemode, readmode

);

  reg [31:0] Mem [2499:250];

  always @(*) begin
    if(writemode)
      Mem[address >> 2] = dataIn;
  end
     assign dataOut = readmode ? Mem[address >> 2] : 0;
endmodule
