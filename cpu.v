module cpu (
    input clk, rst,
    input [31:0] instr,
    input [31:0] b_addr;
    output [31:0] out
    );

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

    //assign b_addr[31:0] = 32'b0000_0000_0000_0000_0000_0000_0000_0000;


    instr_mem instrmem (.address(instr),
                        .instruction(instruction)
                        );

    control_unit contrlu (.op(instruction[31:26]),
                          .regdst(cu_regdst), .regwrite(cu_regwrite), .branch(cu_branch),
                          .jump(cu_jump), .memread(cu_memread), .memtoreg(cu_memtoreg),
                          .memwrite(cu_memwrite), .aluop(cu_aluop), .aluscr(cu_aluscr)
                          );

    mux32 mux1 (regwrite[5:0], instruction[20:16], instruction[15:11], cu_regdst);

    registerMemory regmem ( .reg_read1(instruction[25:21]), .reg_read2(instruction[21:16]),
                          	.reg_write(mux1_regwrite),
                          	.regwrite_con(cu_regwrite),
                          	.write_data(mux3_writedata),
                          	.data1(reg_readdata1),
                          	.data2(reg_readdata2)
                          	);

    sign_extended signext (instruction[15:0], signext_out[32:0]);

    mux32 mux2 (mux2_out, reg_readdata2, signext_out, cu_aluscr);

    alu_control aluctrl (cu_aluop, instruction[5:0], aluctrl_out);

    alu ALU (aluctrl_out, reg_readdata1, mux2_out, alu_out, alu_zero);

    datamem data_mem (dmem_readdata, alu_out, reg_readdata2, cu_memread, cu_memwrite);

    mux32 mux3 (mux3_writedata, dmem_readdata, alu_out);

    reg[31:0] shiftl2_result = signext_out << 2;

    reg[31:0] bBranch;
    and AND(bBranch, cu_branch, alu_zero );

    if (bBranch == 1) {
        out = b_addr;
    }

endmodule
