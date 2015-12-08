module tb_control_unit;
  	reg [5:0] op;
  	wire regdst, regwrite;
	wire branch;
	wire jump;
	wire memread, memtoreg, memwrite;
  	wire [1:0] aluop;
	wire aluscr;

  control_unit cu1 (op, regdst, regwrite, branch, jump, memread, memtoreg, memwrite, aluop, aluscr);

  initial
    begin
      op = 6'b100011;
      $dumpfile("dump.vcd");
      $dumpvars(1);
    end
endmodule;
