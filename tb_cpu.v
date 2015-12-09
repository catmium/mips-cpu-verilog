// Code your testbench here
// or browse Examples
module tb_cpu;
  reg [31:0] addr;
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
  	wire bBranch;

  cpu CPU1(addr,out, instruction, cu_regdst, cu_jump, cu_branch, cu_memread, cu_memtoreg, cu_aluop, cu_memwrite, cu_aluscr, cu_regwrite, mux1_regwrite, mux3_writedata, reg_readdata1, reg_readdata2, signext_out, mux2_out, alu_out, alu_zero, aluctrl_out, dmem_readdata, bBranch);
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(1);
    $monitor("ADDR = %b NEXT ADDR =%b ",addr,out);

    addr = 32'b000000_00000_00000_00000_00000_101000;
    #20
    addr = out;
    #20
    addr = out;
    #20
      $finish;
    end
endmodule
