// Code your testbench here
// or browse Examples
module registerMemory_tb();
	
  	reg[4:0]read_reg1;
  	reg[4:0]read_reg2;
  	reg[4:0]write_reg;
  	reg regwrite_con;
  	reg[31:0]write_data;
  	reg[31:0]data1;
  	reg[31:0]data2;

  
  // call module
  registerMemory tb(read_reg1,read_reg2,write_reg,regwrite_con,write_data,data1,data2);
   
  initial 
    begin
      $dumpfile("dump.vcd");
      $dumpvars(1);
      repeat (100);
    end
  initial begin
      // change input
      read_reg1    = 5'b00000;
      read_reg2    = 5'b01000;
      write_reg    = 5'b01000;
      write_data   = 5'b01111;
      regwrite_con = 1'b1;
      #10
      read_reg1    = 5'b00011;
      read_reg2    = 5'b00000;
      write_reg    = 5'b00011;
      write_data   = 5'b00111;
      regwrite_con = 1'b1;
      #10
      read_reg1    = 5'b00000;
      read_reg2    = 5'b01000;
      write_reg    = 5'b01000;
      write_data   = 5'b01111;
      regwrite_con = 1'b0;
      #10
      $finish;
    end
endmodule