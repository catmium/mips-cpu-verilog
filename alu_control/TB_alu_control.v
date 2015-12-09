module alu_control_t();

  //--- Register Input and output
  reg [1:0] ALUOp;
  reg [5:0] funct;

  wire [3:0] ALUCon;

  //--- call module
  alu_control test(ALUOp, funct, ALUCon);

  initial begin
    //--- dump the file
    $dumpfile("dump.vcd");
      $dumpvars(1);
    //--- change input
    ALUOp = 2'b00;  funct = 6'b100000; #20

    ALUOp = 2'b00;  funct = 6'b110000; #20

    ALUOp = 2'b01;  funct = 6'b101010; #20

    ALUOp = 2'b10;  funct = 6'b100000; #20

    ALUOp = 2'b10;  funct = 6'b100010; #20

    ALUOp = 2'b10;  funct = 6'b100100; #20

    ALUOp = 2'b10;  funct = 6'b100101; #20

    ALUOp = 2'b10;  funct = 6'b101010; #20

    ALUOp = 2'b10;  funct = 6'b100010; #20

    ALUOp = 2'b11;  funct = 6'b100000; #20



    $finish;
  end

endmodule
