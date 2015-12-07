module t_dmem;
  reg [31:0] dataIn  = 32'h00000111;
  reg [31:0] address = 32'h00000003;
  reg [31:0] out;
  reg readmode;
  reg writemode;
  reg Clk=1'b0;

  dmem Datamemory(out,address,dataIn,readmode,writemode);

  always #2 Clk=!Clk;
  initial begin
      $dumpfile("dump.vcd");
      $dumpvars(1);
  end

  initial begin
    writemode = 1'b1;
    readmode = 1'b0;
    #2
    writemode = 1'b0;
    #2
    readmode = 1'b1;
  end
  initial #300 $finish;

endmodule