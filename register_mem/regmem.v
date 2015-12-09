module regmem (
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
	//reg_mem[reg_read1] = 32'b00_0000000000_0000000000_000000100;
	reg_mem[8] = 32'd4; //MEM(1000)
	reg_mem[10] = 32'b00_0000000000_0000000000_0000000010; //data to MEM(1000)
	//reg_mem[10] = 32'b00_0000000000_0000000000_0000000010; //data to MEM(1000)
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
