module alu(
		input		[3:0]	ctl,
		input		[31:0]	a, b,
		output reg	[31:0]	out,
		output				zero);

	wire [31:0] sub_ab;
	wire [31:0] add_ab;
	wire 		slt;

	assign sub_ab = a - b;
	assign add_ab = a + b;

	//assign slt = oflow_sub ? ~(a[31]) : a[31];

	always @(*) begin
		case (ctl)
			4'b0000:  out <= a & b;				/* and */
			4'b0001:  out <= a | b;				/* or */
			4'b0010:  out <= add_ab;				/* add */
			4'b0110:  out <= sub_ab;				/* sub */
			4'b0111: begin
              if (a < b) out = 32'b00000000000000000000000000000001;
              else  out = 32'b00000000000000000000000000000000;
            end	/* slt */

			default: out <= 0;
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
			2'b00: ALUCon <= 4'b0010;
			2'b01: ALUCon <= 4'b0110;
			2'b10: begin
                      if(funct == 6'b100000)
                        ALUCon <= 4'b0010;
                      else if(funct == 6'b100010)
                        ALUCon <= 4'b0110;
                      else if(funct == 6'b100101)
                        ALUCon <= 4'b0001;
                      else if(funct == 6'b100100)
                        ALUCon <= 4'b0000;
                      else if(funct == 6'b101010)
                        ALUCon <= 4'b0111;
                      else
                        ALUCon <= 4'b0000;
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
          	memory[10] = 32'b001000_00000_01000_00000_00000_000101;    // addi $t0, $0, 5
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
input signed [15:0] in,
output signed [31:0] out);

assign out = in;

endmodule

module cpu (
    //input clk, rst
  	input [31:0] addr,
  	//input [31:0] b_addr,
    output reg [31:0] out
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
  	reg bBranch;


    instr_mem instrmem (addr,instruction );

    control_unit contrlu (.op(instruction[31:26]),
                          .regdst(cu_regdst), .regwrite(cu_regwrite), .branch(cu_branch),
                          .jump(cu_jump), .memread(cu_memread), .memtoreg(cu_memtoreg),
                          .memwrite(cu_memwrite), .aluop(cu_aluop), .aluscr(cu_aluscr)
                          );

  	mux5 mux1 (mux1_regwrite, instruction[20:16], instruction[15:11], cu_regdst);

  	registerMemory regmem ( .reg_read1(instruction[25:21]), .reg_read2(instruction[20:16]),
                           .reg_write(mux1_regwrite[4:0]),
                          	.regwrite_con(cu_regwrite),
                          	.write_data(mux3_writedata),
                          	.data1(reg_readdata1),
                          	.data2(reg_readdata2)
                          	);

  	sign_extended signext (instruction[15:0], signext_out[31:0]);

    mux32 mux2 (mux2_out, reg_readdata2, signext_out, cu_aluscr);

    alu_control aluctrl (cu_aluop, instruction[5:0], aluctrl_out);

    alu ALU (aluctrl_out, reg_readdata1, mux2_out, alu_out, alu_zero);

    datamem data_mem (dmem_readdata, alu_out, reg_readdata2, cu_memread, cu_memwrite);

  	mux32 mux3(mux3_writedata, dmem_readdata, alu_out, cu_memtoreg);

    reg[31:0] shiftl2_result = signext_out << 2;

    and AND1(bBranch, cu_branch, alu_zero );

    /*always @(*) begin

  	if (bBranch == 1'b1)
      out = instruction[31:0] + b_addr + 32'b0000_0000_0000_0000_0000_0000_0000_0100;
    else
      assign out = instruction[31:0] + 32'b0000_0000_0000_0000_0000_0000_0000_0100;
    end*/

endmodule 
