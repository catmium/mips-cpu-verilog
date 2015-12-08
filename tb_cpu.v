// Code your testbench here
// or browse Examples

module tb_cpu;
  //input clk, rst;
  reg [31:0] addr;
  //reg [31:0] b_addr;
  wire [31:0] out;

  	wire[31:0] instruction;
    wire cu_regdst, cu_jump, cu_branch, cu_memread, cu_memtoreg;
    wire[1:0] cu_aluop;
    wire cu_memwrite, cu_aluscr, cu_regwrite;
    wire[4:0] mux1_regwrite;
    wire[31:0] mux3_writedata;
    wire[31:0] reg_readdata1, reg_readdata2;
    wire[31:0] signext_out;
    wire[31:0] mux2_out;
    wire[31:0] alu_out;
    wire alu_zero;
    wire[3:0] aluctrl_out;
    wire[31:0] dmem_readdata;

  cpu CPU1(addr, out);
  initial
    begin

      $display("RESULT");
      //$monitor("PC = %b  Next PC =%b ",instr,out);
      addr = 32'b000000_00000_00000_00000_00000_101000;
      $dumpfile("dump.vcd");
      $dumpvars(1);
    end
endmodule
