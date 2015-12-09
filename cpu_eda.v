module alu(
		input		[3:0]	ctl,
		input		[31:0]	a, b,
		output reg	[31:0]	out,
		output				zero);

	wire [31:0] sub_ab;
	wire [31:0] add_ab;

	//assign sub_ab = a - b;
	//assign add_ab = a + b;

	//assign slt = oflow_sub ? ~(a[31]) : a[31];

	always @(*) begin
		case (ctl)
			4'b0000:  out = a & b;				/* and */
			4'b0001:  out = a | b;				/* or */
			4'b0010:  out = a + b;				/* add */
			4'b0110:  out = a - b;				/* sub */
            4'b0111:  out = (a < b) ? 32'b00000000000000000000000000000001 : 32'b0; //slt
			default: out = 0;
		endcase
	end
  	assign zero = (0 == out)? 1 : out;
endmodule


module alu_control(ALUOp,funct,ALUCon);
 input [1:0] ALUOp;
 input [5:0] funct;
 output [3:0] ALUCon;

reg [3:0] ALUCon;

	always @(*) begin
		case(ALUOp)
			2'b00: ALUCon <= 4'b0010;    // lw, sw
			2'b01: ALUCon <= 4'b0110;    // beq
            2'b10 :
              begin
                case (funct)
                  6'b100000: ALUCon <= 4'b0010;
                  6'b100010: ALUCon <= 4'b0110;
                  6'b100101: ALUCon <= 4'b0001;
                  6'b100100: ALUCon <= 4'b0000;
                  6'b101010: ALUCon <= 4'b0111;
                  default: 	 ALUCon <= 4'b0000;
                endcase
              end
			default: ALUCon <= 4'b0000;
		endcase
	end

endmodule

module control_unit (
	input[5:0] 	op,
	output	reg regdst, regwrite,
	output 	reg branch,
	output	reg jump,
	output	reg memread, memtoreg, memwrite,
	output	reg [1:0] aluop,
	output	reg aluscr );

	always @(*) begin
		// default
		// R-type
		branch 		<= 1'b0;
		jump 		<= 1'b0;
		memread 	<= 1'b0;
		memtoreg	<= 1'b0;
		memwrite	<= 1'b0;
		aluop[1:0]	<= 2'b10;
		aluscr		<= 1'b0;
		regdst 		<= 1'b1;
		regwrite	<= 1'b1;

		case(op)
			6'b10_0011: begin // lw
				regdst		<= 1'b0;
				memread		<= 1'b1;
              aluop[1:0]	<= 2'b00;
				aluscr		<= 1'b1;
              	memtoreg	<= 1'b1;
			end
			6'b10_1011: begin // sw
              	aluop[1]	<= 1'b0;
				aluscr		<= 1'b1;
				memwrite 	<= 1'b1;
				regwrite	<= 1'b0;
			end
			6'b00_0100: begin // beq
				branch		<= 1'b1;
				aluop[1:0]	<= 2'b01;
				regwrite	<= 1'b0;
			end
			6'b00_1000: begin // addi
				regdst   	<= 1'b0;
				aluop[1] 	<= 1'b0;
				aluscr   	<= 1'b1;
			end
			6'b00_0000: begin // add
			end
		endcase
	end
endmodule


module datamem(dataOut, address, dataIn, readmode, writemode);
    output reg [31:0] dataOut;
    input [31:0] address;
    input [31:0] dataIn;
    input readmode;
    input writemode;

    reg [31:0] dMemory [511:0];

    //do when read or write signal is recieved
    always@ (readmode or writemode)
    begin
        if (writemode == 1)
            dMemory[address]=dataIn; //store data
        if (readmode == 1)
            dataOut = dMemory[address]; //load data
    end

endmodule

module instr_mem (	input [31:0] address,
					output [31:0] instruction );

 	reg [31:0] memory [249:0];
 	integer i;

	initial
		begin
			for (i=0; i<250; i=i+1)
              memory[i] = 32'b0;
 				// Insert MIPS's assembly here start at memory[10] = ...
          memory[10] = 32'b000000_01000_01001_01010_00000_100000;    // addi $t0, $0, 5
		end

	assign instruction = memory[address >> 2];

endmodule


module mux32(out, in0, in1, sel);
    output reg  [31:0] out;
    input [31:0] in0;
    input [31:0] in1;
	input sel;    //select which inputs to send out

  	always @(sel)
    begin
      if (sel==0)
            out = in0;
      else
            out = in1;
    end
endmodule

module mux5(out, in0, in1, sel);
  output reg  [4:0] out;
  input [4:0] in0;
  input [4:0] in1;
	input sel;    //select which inputs to send out

  	always @(sel)
    begin
      if (sel==0)
            out = in0;
      else
            out = in1;
    end
endmodule

module registerMemory (
	input[4:0]reg_read1,
	input[4:0]reg_read2,
	input[4:0]reg_write,
	input regwrite_con, //register wirte from control unit
  	input[31:0] write_data,
	output reg[31:0] data1,
	output reg[31:0] data2
	);

 	reg[31:0]reg_mem[31:0];
  	integer i;
  	initial
      begin
  for (i=0;i<32;i++) reg_mem[i] = 0;
    reg_mem[reg_read1] = 32'b00_0000000000_0000000000_000000100;
    reg_mem[reg_read2] = 32'b00_0000000000_0000000000_000000010;
  end
 	always @(*) begin
		if (reg_read1 == 0)
				data1 = 0;
		else if ((reg_read1 == reg_write) && regwrite_con)
				data1 = write_data;
		else
				data1 = reg_mem[reg_read1][31:0];
	end

	always @(*) begin
		if (reg_read2 == 0)
				data2 = 0;
		else if ((reg_read2 == reg_write) && regwrite_con)
				data2 = write_data;
		else
				data2 = reg_mem[reg_read2][31:0];
	end

	always @(*) begin
		if (regwrite_con && reg_write != 0)
			// write a non $zero register
          reg_mem[reg_write] <= write_data;
	end
endmodule

module sign_extended (
  output signed [31:0] out,
input signed [15:0] in);

assign out = in;

endmodule

module cpu (
    //input clk, rst
  	input [31:0] addr,
  	//input [31:0] b_addr,
  output reg [31:0] out,
  output reg [31:0] instruction,
  output reg cu_regdst, cu_jump, cu_branch, cu_memread, cu_memtoreg,
  output reg[1:0] cu_aluop,
  output reg cu_memwrite, cu_aluscr, cu_regwrite,
  output reg[4:0] mux1_regwrite,
  output reg[31:0] mux3_writedata,
  output reg[31:0] reg_readdata1, reg_readdata2,
  output reg[31:0] signext_out,
  output reg[31:0] mux2_out,
  output reg[31:0] alu_out,
  output reg alu_zero,
  output reg[3:0] aluctrl_out,
  output reg[31:0] dmem_readdata,
  output reg bBranch
    );

    //wire[31:0] instruction;
    /*wire cu_regdst, cu_jump, cu_branch, cu_memread, cu_memtoreg;
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
  	reg bBranch;*/

    instr_mem instrmem (addr,instruction );

  	///assign out = instruction;

    control_unit contrlu (instruction[31:26],
                          cu_regdst, cu_regwrite, cu_branch,
                          cu_jump, cu_memread, cu_memtoreg,
                          cu_memwrite, cu_aluop, cu_aluscr );

  	mux5 mux01 (mux1_regwrite, instruction[20:16], instruction[15:11], cu_regdst);

  	registerMemory regmem ( .reg_read1(instruction[25:21]), .reg_read2(instruction[20:16]),
                           .reg_write(mux1_regwrite[4:0]),
                          	.regwrite_con(cu_regwrite),
                          	.write_data(mux3_writedata),
                          	.data1(reg_readdata1),
                          	.data2(reg_readdata2)
                          	);

  	sign_extended signext (signext_out[31:0], instruction[15:0]);

  	mux32 mux02 (mux2_out, reg_readdata2, signext_out, cu_aluscr);

    alu_control aluctrl (cu_aluop, instruction[5:0], aluctrl_out);

    alu ALU (aluctrl_out, reg_readdata1, mux2_out, alu_out, alu_zero);

    datamem data_mem (dmem_readdata, alu_out, reg_readdata2, cu_memread, cu_memwrite);

  	mux32 mux03(mux3_writedata, alu_out, dmem_readdata, cu_memtoreg);

    and AND1(bBranch, cu_branch, alu_zero );

  	reg[31:0] branch_shiftl2_result = signext_out << 2;

  	reg[31:0] pcp4 = addr + 32'b0000_0000_0000_0000_0000_0000_0000_0100;
  	reg[31:0] j_addr = {pcp4[31:28],(instruction[25:0] << 2)};

  	reg[31:0] b_addr = instruction[31:0] + branch_shiftl2_result + 32'b0000_0000_0000_0000_0000_0000_0000_0100;

  	reg[31:0] mux04_result;

  	mux32 mux04 (mux04_result, pcp4, b_addr, bBranch);
  	mux32 mux05 (mux05_result, mux04_result, j_addr, bBranch);

    out = mux05_result;

endmodule
