module MyClockGen (
	input_clk_25MHz,
	clk_proc,
	locked
);
	input input_clk_25MHz;
	output wire clk_proc;
	output wire locked;
	wire clkfb;
	(* FREQUENCY_PIN_CLKI = "25" *) (* FREQUENCY_PIN_CLKOP = "20" *) (* ICP_CURRENT = "12" *) (* LPF_RESISTOR = "8" *) (* MFG_ENABLE_FILTEROPAMP = "1" *) (* MFG_GMCREF_SEL = "2" *) EHXPLLL #(
		.PLLRST_ENA("DISABLED"),
		.INTFB_WAKE("DISABLED"),
		.STDBY_ENABLE("DISABLED"),
		.DPHASE_SOURCE("DISABLED"),
		.OUTDIVIDER_MUXA("DIVA"),
		.OUTDIVIDER_MUXB("DIVB"),
		.OUTDIVIDER_MUXC("DIVC"),
		.OUTDIVIDER_MUXD("DIVD"),
		.CLKI_DIV(5),
		.CLKOP_ENABLE("ENABLED"),
		.CLKOP_DIV(30),
		.CLKOP_CPHASE(15),
		.CLKOP_FPHASE(0),
		.FEEDBK_PATH("INT_OP"),
		.CLKFB_DIV(4)
	) pll_i(
		.RST(1'b0),
		.STDBY(1'b0),
		.CLKI(input_clk_25MHz),
		.CLKOP(clk_proc),
		.CLKFB(clkfb),
		.CLKINTFB(clkfb),
		.PHASESEL0(1'b0),
		.PHASESEL1(1'b0),
		.PHASEDIR(1'b1),
		.PHASESTEP(1'b1),
		.PHASELOADREG(1'b1),
		.PLLWAKESYNC(1'b0),
		.ENCLKOP(1'b0),
		.LOCK(locked)
	);
endmodule
module gp1 (
	a,
	b,
	g,
	p
);
	input wire a;
	input wire b;
	output wire g;
	output wire p;
	assign g = a & b;
	assign p = a | b;
endmodule
module gp4 (
	gin,
	pin,
	cin,
	gout,
	pout,
	cout
);
	input wire [3:0] gin;
	input wire [3:0] pin;
	input wire cin;
	output wire gout;
	output wire pout;
	output wire [2:0] cout;
	assign gout = ((gin[3] | (pin[3] & gin[2])) | (&pin[3:2] & gin[1])) | (&pin[3:1] & gin[0]);
	assign pout = &pin;
	wire c1 = gin[0] | (pin[0] & cin);
	wire c2 = gin[1] | (pin[1] & c1);
	wire c3 = gin[2] | (pin[2] & c2);
	assign cout = {c3, c2, c1};
endmodule
module gp8 (
	gin,
	pin,
	cin,
	gout,
	pout,
	cout
);
	input wire [7:0] gin;
	input wire [7:0] pin;
	input wire cin;
	output wire gout;
	output wire pout;
	output wire [6:0] cout;
	wire g0out;
	wire p0out;
	wire [2:0] c0out;
	gp4 a(
		.gin(gin[3:0]),
		.pin(pin[3:0]),
		.cin(cin),
		.gout(g0out),
		.pout(p0out),
		.cout(c0out)
	);
	wire c4 = g0out | (p0out & cin);
	wire g1out;
	wire p1out;
	wire [2:0] c1out;
	gp4 b(
		.gin(gin[7:4]),
		.pin(pin[7:4]),
		.cin(c4),
		.gout(g1out),
		.pout(p1out),
		.cout(c1out)
	);
	assign gout = g1out | (p1out & g0out);
	assign pout = p1out & p0out;
	assign cout = {c1out, c4, c0out};
endmodule
module CarryLookaheadAdder (
	a,
	b,
	cin,
	sum
);
	input wire [31:0] a;
	input wire [31:0] b;
	input wire cin;
	output wire [31:0] sum;
	wire [31:0] g;
	wire [31:0] p;
	genvar _gv_i_1;
	generate
		for (_gv_i_1 = 0; _gv_i_1 < 32; _gv_i_1 = _gv_i_1 + 1) begin : genblk1
			localparam i = _gv_i_1;
			gp1 gp(
				.a(a[i]),
				.b(b[i]),
				.g(g[i]),
				.p(p[i])
			);
		end
	endgenerate
	wire [3:0] g8out;
	wire [3:0] p8out;
	wire [6:0] c8_0;
	gp8 gp_0(
		.gin(g[7:0]),
		.pin(p[7:0]),
		.cin(cin),
		.gout(g8out[0]),
		.pout(p8out[0]),
		.cout(c8_0)
	);
	wire cin_1 = g8out[0] | (p8out[0] & cin);
	wire [6:0] c8_1;
	gp8 gp_1(
		.gin(g[15:8]),
		.pin(p[15:8]),
		.cin(cin_1),
		.gout(g8out[1]),
		.pout(p8out[1]),
		.cout(c8_1)
	);
	wire cin_2 = g8out[1] | (p8out[1] & cin_1);
	wire [6:0] c8_2;
	gp8 gp_2(
		.gin(g[23:16]),
		.pin(p[23:16]),
		.cin(cin_2),
		.gout(g8out[2]),
		.pout(p8out[2]),
		.cout(c8_2)
	);
	wire cin_3 = g8out[2] | (p8out[2] & cin_2);
	wire [6:0] c8_3;
	gp8 gp_3(
		.gin(g[31:24]),
		.pin(p[31:24]),
		.cin(cin_3),
		.gout(g8out[3]),
		.pout(p8out[3]),
		.cout(c8_3)
	);
	wire [31:0] c = {c8_3, cin_3, c8_2, cin_2, c8_1, cin_1, c8_0, cin};
	assign sum = (a ^ b) ^ c;
endmodule
module Disasm (
	insn,
	disasm
);
	parameter signed [7:0] PREFIX = "D";
	input wire [31:0] insn;
	output wire [255:0] disasm;
endmodule
module RegFile (
	rd,
	rd_data,
	rs1,
	rs1_data,
	rs2,
	rs2_data,
	clk,
	we,
	rst
);
	input wire [4:0] rd;
	input wire [31:0] rd_data;
	input wire [4:0] rs1;
	output wire [31:0] rs1_data;
	input wire [4:0] rs2;
	output wire [31:0] rs2_data;
	input wire clk;
	input wire we;
	input wire rst;
	localparam signed [31:0] NumRegs = 32;
	reg [31:0] regs [0:31];
	wire [32:1] sv2v_tmp_E4190;
	assign sv2v_tmp_E4190 = 32'd0;
	always @(*) regs[0] = sv2v_tmp_E4190;
	assign rs1_data = regs[rs1];
	assign rs2_data = regs[rs2];
	genvar _gv_i_3;
	function automatic signed [4:0] sv2v_cast_5_signed;
		input reg signed [4:0] inp;
		sv2v_cast_5_signed = inp;
	endfunction
	generate
		for (_gv_i_3 = 1; _gv_i_3 < 32; _gv_i_3 = _gv_i_3 + 1) begin : genblk1
			localparam i = _gv_i_3;
			always @(posedge clk)
				if (rst)
					regs[i] <= 32'd0;
				else if (we && (rd == sv2v_cast_5_signed(i)))
					regs[i] <= rd_data;
		end
	endgenerate
endmodule
module DatapathPipelined (
	clk,
	rst,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem,
	halt,
	trace_completed_pc,
	trace_completed_insn,
	trace_completed_cycle_status
);
	reg _sv2v_0;
	input wire clk;
	input wire rst;
	output wire [31:0] pc_to_imem;
	input wire [31:0] insn_from_imem;
	output wire [31:0] addr_to_dmem;
	input wire [31:0] load_data_from_dmem;
	output wire [31:0] store_data_to_dmem;
	output wire [3:0] store_we_to_dmem;
	output wire halt;
	output wire [31:0] trace_completed_pc;
	output wire [31:0] trace_completed_insn;
	output wire [31:0] trace_completed_cycle_status;
	localparam [6:0] OpcodeLoad = 7'b0000011;
	localparam [6:0] OpcodeStore = 7'b0100011;
	localparam [6:0] OpcodeBranch = 7'b1100011;
	localparam [6:0] OpcodeJalr = 7'b1100111;
	localparam [6:0] OpcodeMiscMem = 7'b0001111;
	localparam [6:0] OpcodeJal = 7'b1101111;
	localparam [6:0] OpcodeRegImm = 7'b0010011;
	localparam [6:0] OpcodeRegReg = 7'b0110011;
	localparam [6:0] OpcodeEnviron = 7'b1110011;
	localparam [6:0] OpcodeAuipc = 7'b0010111;
	localparam [6:0] OpcodeLui = 7'b0110111;
	reg [31:0] cycles_current;
	always @(posedge clk)
		if (rst)
			cycles_current <= 0;
		else
			cycles_current <= cycles_current + 1;
	wire [6:0] insn_funct7;
	wire [4:0] insn_rs2;
	wire [4:0] insn_rs1;
	wire [2:0] insn_funct3;
	wire [4:0] insn_rd;
	wire [6:0] insn_opcode;
	wire [11:0] imm_i;
	wire [4:0] imm_shamt;
	wire [11:0] imm_s;
	wire [12:0] imm_b;
	wire [19:0] imm_u;
	wire [31:0] imm_i_sext;
	wire [31:0] imm_s_sext;
	wire [31:0] imm_b_sext;
	reg branch_taken;
	reg [31:0] branch_target_actual;
	reg [31:0] f_pc_current;
	reg [31:0] f_pc_sent;
	wire [31:0] f_insn;
	reg [31:0] f_cycle_status;
	reg [31:0] f_pc_next;
	always @(*) begin
		if (_sv2v_0)
			;
		f_pc_next = f_pc_current + 4;
		if (branch_taken)
			f_pc_next = branch_target_actual;
	end
	always @(posedge clk)
		if (rst) begin
			f_pc_current <= 32'd0;
			f_pc_sent <= 32'd0;
			f_cycle_status <= 32'd1;
		end
		else begin
			f_pc_sent <= f_pc_current;
			f_pc_current <= f_pc_next;
			f_cycle_status <= 32'd1;
		end
	assign pc_to_imem = f_pc_current;
	assign f_insn = insn_from_imem;
	wire [255:0] f_disasm;
	Disasm #(.PREFIX("F")) disasm_0fetch(
		.insn(f_insn),
		.disasm(f_disasm)
	);
	reg [95:0] decode_state;
	reg [95:0] decode_next;
	wire [31:0] d_rs1_data;
	wire [31:0] d_rs2_data;
	reg [31:0] d_rs1_data_bypassed;
	reg [31:0] d_rs2_data_bypassed;
	reg [133:0] writeback_state;
	always @(*) begin
		if (_sv2v_0)
			;
		d_rs1_data_bypassed = d_rs1_data;
		d_rs2_data_bypassed = d_rs2_data;
		if (writeback_state[32] && (writeback_state[37-:5] != 5'd0)) begin
			if (writeback_state[37-:5] == insn_rs1)
				d_rs1_data_bypassed = writeback_state[31-:32];
			if (writeback_state[37-:5] == insn_rs2)
				d_rs2_data_bypassed = writeback_state[31-:32];
		end
	end
	assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = decode_state[63-:32];
	assign imm_u = decode_state[63:44];
	assign imm_i = decode_state[63:52];
	assign imm_shamt = decode_state[56:52];
	assign imm_s = {insn_funct7, insn_rd};
	assign {imm_b[12], imm_b[10:5]} = insn_funct7;
	assign {imm_b[4:1], imm_b[11]} = insn_rd;
	assign imm_b[0] = 1'b0;
	assign imm_i_sext = {{20 {imm_i[11]}}, imm_i};
	assign imm_s_sext = {{20 {imm_s[11]}}, imm_s};
	assign imm_b_sext = {{19 {imm_b[12]}}, imm_b};
	RegFile rf(
		.clk(clk),
		.rst(rst),
		.we(writeback_state[32]),
		.rd(writeback_state[37-:5]),
		.rd_data(writeback_state[31-:32]),
		.rs1(insn_rs1),
		.rs2(insn_rs2),
		.rs1_data(d_rs1_data),
		.rs2_data(d_rs2_data)
	);
	wire flush_decode;
	assign flush_decode = branch_taken;
	always @(*) begin
		if (_sv2v_0)
			;
		if (flush_decode)
			decode_next = 96'h000000000000000000000008;
		else
			decode_next = {f_pc_current, f_insn, f_cycle_status};
	end
	always @(posedge clk)
		if (rst)
			decode_state <= 96'h000000000000000000000004;
		else
			decode_state <= decode_next;
	wire [255:0] d_disasm;
	Disasm #(.PREFIX("D")) disasm_1decode(
		.insn(decode_state[63-:32]),
		.disasm(d_disasm)
	);
	reg [239:0] execute_state;
	reg [239:0] execute_next;
	wire insn_lui = insn_opcode == OpcodeLui;
	wire insn_beq = (insn_opcode == OpcodeBranch) && (decode_state[46:44] == 3'b000);
	wire insn_bne = (insn_opcode == OpcodeBranch) && (decode_state[46:44] == 3'b001);
	wire insn_blt = (insn_opcode == OpcodeBranch) && (decode_state[46:44] == 3'b100);
	wire insn_bge = (insn_opcode == OpcodeBranch) && (decode_state[46:44] == 3'b101);
	wire insn_bltu = (insn_opcode == OpcodeBranch) && (decode_state[46:44] == 3'b110);
	wire insn_bgeu = (insn_opcode == OpcodeBranch) && (decode_state[46:44] == 3'b111);
	wire insn_addi = (insn_opcode == OpcodeRegImm) && (decode_state[46:44] == 3'b000);
	wire insn_slti = (insn_opcode == OpcodeRegImm) && (decode_state[46:44] == 3'b010);
	wire insn_sltiu = (insn_opcode == OpcodeRegImm) && (decode_state[46:44] == 3'b011);
	wire insn_xori = (insn_opcode == OpcodeRegImm) && (decode_state[46:44] == 3'b100);
	wire insn_ori = (insn_opcode == OpcodeRegImm) && (decode_state[46:44] == 3'b110);
	wire insn_andi = (insn_opcode == OpcodeRegImm) && (decode_state[46:44] == 3'b111);
	wire insn_slli = ((insn_opcode == OpcodeRegImm) && (decode_state[46:44] == 3'b001)) && (decode_state[63:57] == 7'd0);
	wire insn_srli = ((insn_opcode == OpcodeRegImm) && (decode_state[46:44] == 3'b101)) && (decode_state[63:57] == 7'd0);
	wire insn_srai = ((insn_opcode == OpcodeRegImm) && (decode_state[46:44] == 3'b101)) && (decode_state[63:57] == 7'b0100000);
	wire insn_add = ((insn_opcode == OpcodeRegReg) && (decode_state[46:44] == 3'b000)) && (decode_state[63:57] == 7'd0);
	wire insn_sub = ((insn_opcode == OpcodeRegReg) && (decode_state[46:44] == 3'b000)) && (decode_state[63:57] == 7'b0100000);
	wire insn_sll = ((insn_opcode == OpcodeRegReg) && (decode_state[46:44] == 3'b001)) && (decode_state[63:57] == 7'd0);
	wire insn_slt = ((insn_opcode == OpcodeRegReg) && (decode_state[46:44] == 3'b010)) && (decode_state[63:57] == 7'd0);
	wire insn_sltu = ((insn_opcode == OpcodeRegReg) && (decode_state[46:44] == 3'b011)) && (decode_state[63:57] == 7'd0);
	wire insn_xor = ((insn_opcode == OpcodeRegReg) && (decode_state[46:44] == 3'b100)) && (decode_state[63:57] == 7'd0);
	wire insn_srl = ((insn_opcode == OpcodeRegReg) && (decode_state[46:44] == 3'b101)) && (decode_state[63:57] == 7'd0);
	wire insn_sra = ((insn_opcode == OpcodeRegReg) && (decode_state[46:44] == 3'b101)) && (decode_state[63:57] == 7'b0100000);
	wire insn_or = ((insn_opcode == OpcodeRegReg) && (decode_state[46:44] == 3'b110)) && (decode_state[63:57] == 7'd0);
	wire insn_and = ((insn_opcode == OpcodeRegReg) && (decode_state[46:44] == 3'b111)) && (decode_state[63:57] == 7'd0);
	wire insn_ecall = (insn_opcode == OpcodeEnviron) && (decode_state[63:39] == 25'd0);
	reg [165:0] memory_state;
	reg [31:0] e_rs1_val;
	reg [31:0] e_rs2_val;
	always @(*) begin
		if (_sv2v_0)
			;
		e_rs1_val = execute_state[143-:32];
		e_rs2_val = execute_state[111-:32];
		if (writeback_state[32] && (writeback_state[37-:5] != 5'd0)) begin
			if (writeback_state[37-:5] == execute_state[42-:5])
				e_rs1_val = writeback_state[31-:32];
			if (writeback_state[37-:5] == execute_state[37-:5])
				e_rs2_val = writeback_state[31-:32];
		end
		if (memory_state[32] && (memory_state[37-:5] != 5'd0)) begin
			if (memory_state[37-:5] == execute_state[42-:5])
				e_rs1_val = memory_state[69-:32];
			if (memory_state[37-:5] == execute_state[37-:5])
				e_rs2_val = memory_state[69-:32];
		end
	end
	wire e_insn_lui = execute_state[182:176] == OpcodeLui;
	wire e_insn_addi = (execute_state[182:176] == OpcodeRegImm) && (execute_state[190:188] == 3'b000);
	wire e_insn_slti = (execute_state[182:176] == OpcodeRegImm) && (execute_state[190:188] == 3'b010);
	wire e_insn_sltiu = (execute_state[182:176] == OpcodeRegImm) && (execute_state[190:188] == 3'b011);
	wire e_insn_xori = (execute_state[182:176] == OpcodeRegImm) && (execute_state[190:188] == 3'b100);
	wire e_insn_ori = (execute_state[182:176] == OpcodeRegImm) && (execute_state[190:188] == 3'b110);
	wire e_insn_andi = (execute_state[182:176] == OpcodeRegImm) && (execute_state[190:188] == 3'b111);
	wire e_insn_slli = ((execute_state[182:176] == OpcodeRegImm) && (execute_state[190:188] == 3'b001)) && (execute_state[207:201] == 7'd0);
	wire e_insn_srli = ((execute_state[182:176] == OpcodeRegImm) && (execute_state[190:188] == 3'b101)) && (execute_state[207:201] == 7'd0);
	wire e_insn_srai = ((execute_state[182:176] == OpcodeRegImm) && (execute_state[190:188] == 3'b101)) && (execute_state[207:201] == 7'b0100000);
	wire e_insn_add = ((execute_state[182:176] == OpcodeRegReg) && (execute_state[190:188] == 3'b000)) && (execute_state[207:201] == 7'd0);
	wire e_insn_sub = ((execute_state[182:176] == OpcodeRegReg) && (execute_state[190:188] == 3'b000)) && (execute_state[207:201] == 7'b0100000);
	wire e_insn_sll = ((execute_state[182:176] == OpcodeRegReg) && (execute_state[190:188] == 3'b001)) && (execute_state[207:201] == 7'd0);
	wire e_insn_slt = ((execute_state[182:176] == OpcodeRegReg) && (execute_state[190:188] == 3'b010)) && (execute_state[207:201] == 7'd0);
	wire e_insn_sltu = ((execute_state[182:176] == OpcodeRegReg) && (execute_state[190:188] == 3'b011)) && (execute_state[207:201] == 7'd0);
	wire e_insn_xor = ((execute_state[182:176] == OpcodeRegReg) && (execute_state[190:188] == 3'b100)) && (execute_state[207:201] == 7'd0);
	wire e_insn_srl = ((execute_state[182:176] == OpcodeRegReg) && (execute_state[190:188] == 3'b101)) && (execute_state[207:201] == 7'd0);
	wire e_insn_sra = ((execute_state[182:176] == OpcodeRegReg) && (execute_state[190:188] == 3'b101)) && (execute_state[207:201] == 7'b0100000);
	wire e_insn_or = ((execute_state[182:176] == OpcodeRegReg) && (execute_state[190:188] == 3'b110)) && (execute_state[207:201] == 7'd0);
	wire e_insn_and = ((execute_state[182:176] == OpcodeRegReg) && (execute_state[190:188] == 3'b111)) && (execute_state[207:201] == 7'd0);
	reg [31:0] alu_a;
	reg [31:0] alu_b;
	wire [31:0] alu_result;
	reg alu_cin;
	CarryLookaheadAdder alu_adder(
		.a(alu_a),
		.b(alu_b),
		.cin(alu_cin),
		.sum(alu_result)
	);
	reg [31:0] e_rd_data;
	always @(*) begin
		if (_sv2v_0)
			;
		alu_a = 32'd0;
		alu_b = 32'd0;
		alu_cin = 1'b0;
		e_rd_data = 32'd0;
		if (e_insn_lui)
			e_rd_data = execute_state[79-:32];
		else if (e_insn_addi) begin
			alu_a = e_rs1_val;
			alu_b = execute_state[79-:32];
			e_rd_data = alu_result;
		end
		else if (e_insn_slti)
			e_rd_data = ($signed(e_rs1_val) < $signed(execute_state[79-:32]) ? 32'b00000000000000000000000000000001 : 32'b00000000000000000000000000000000);
		else if (e_insn_sltiu)
			e_rd_data = ($unsigned(e_rs1_val) < $unsigned(execute_state[79-:32]) ? 32'b00000000000000000000000000000001 : 32'b00000000000000000000000000000000);
		else if (e_insn_xori)
			e_rd_data = e_rs1_val ^ execute_state[79-:32];
		else if (e_insn_ori)
			e_rd_data = e_rs1_val | execute_state[79-:32];
		else if (e_insn_andi)
			e_rd_data = e_rs1_val & execute_state[79-:32];
		else if (e_insn_slli)
			e_rd_data = e_rs1_val << execute_state[52:48];
		else if (e_insn_srli)
			e_rd_data = e_rs1_val >> execute_state[52:48];
		else if (e_insn_srai)
			e_rd_data = $signed(e_rs1_val) >>> execute_state[52:48];
		else if (e_insn_add) begin
			alu_a = e_rs1_val;
			alu_b = e_rs2_val;
			e_rd_data = alu_result;
		end
		else if (e_insn_sub) begin
			alu_a = e_rs1_val;
			alu_b = ~e_rs2_val;
			alu_cin = 1'b1;
			e_rd_data = alu_result;
		end
		else if (e_insn_sll)
			e_rd_data = e_rs1_val << e_rs2_val[4:0];
		else if (e_insn_slt)
			e_rd_data = ($signed(e_rs1_val) < $signed(e_rs2_val) ? 32'b00000000000000000000000000000001 : 32'b00000000000000000000000000000000);
		else if (e_insn_sltu)
			e_rd_data = ($unsigned(e_rs1_val) < $unsigned(e_rs2_val) ? 32'b00000000000000000000000000000001 : 32'b00000000000000000000000000000000);
		else if (e_insn_xor)
			e_rd_data = e_rs1_val ^ e_rs2_val;
		else if (e_insn_srl)
			e_rd_data = e_rs1_val >> e_rs2_val[4:0];
		else if (e_insn_sra)
			e_rd_data = $signed(e_rs1_val) >>> e_rs2_val[4:0];
		else if (e_insn_or)
			e_rd_data = e_rs1_val | e_rs2_val;
		else if (e_insn_and)
			e_rd_data = e_rs1_val & e_rs2_val;
	end
	wire e_insn_beq = (execute_state[182:176] == OpcodeBranch) && (execute_state[190:188] == 3'b000);
	wire e_insn_bne = (execute_state[182:176] == OpcodeBranch) && (execute_state[190:188] == 3'b001);
	wire e_insn_blt = (execute_state[182:176] == OpcodeBranch) && (execute_state[190:188] == 3'b100);
	wire e_insn_bge = (execute_state[182:176] == OpcodeBranch) && (execute_state[190:188] == 3'b101);
	wire e_insn_bltu = (execute_state[182:176] == OpcodeBranch) && (execute_state[190:188] == 3'b110);
	wire e_insn_bgeu = (execute_state[182:176] == OpcodeBranch) && (execute_state[190:188] == 3'b111);
	wire [12:0] e_imm_b;
	assign {e_imm_b[12], e_imm_b[10:5]} = execute_state[207:201];
	assign {e_imm_b[4:1], e_imm_b[11]} = execute_state[187:183];
	assign e_imm_b[0] = 1'b0;
	wire [31:0] e_branch_target;
	CarryLookaheadAdder e_branch_adder(
		.a(execute_state[239-:32]),
		.b({{19 {e_imm_b[12]}}, e_imm_b[12:0]}),
		.cin(1'b0),
		.sum(e_branch_target)
	);
	always @(*) begin
		if (_sv2v_0)
			;
		branch_taken = 1'b0;
		branch_target_actual = e_branch_target;
		if (e_insn_beq)
			branch_taken = e_rs1_val == e_rs2_val;
		else if (e_insn_bne)
			branch_taken = e_rs1_val != e_rs2_val;
		else if (e_insn_blt)
			branch_taken = $signed(e_rs1_val) < $signed(e_rs2_val);
		else if (e_insn_bge)
			branch_taken = $signed(e_rs1_val) >= $signed(e_rs2_val);
		else if (e_insn_bltu)
			branch_taken = $unsigned(e_rs1_val) < $unsigned(e_rs2_val);
		else if (e_insn_bgeu)
			branch_taken = $unsigned(e_rs1_val) >= $unsigned(e_rs2_val);
	end
	reg d_we;
	reg [31:0] d_imm_val;
	always @(*) begin
		if (_sv2v_0)
			;
		d_we = 1'b0;
		d_imm_val = imm_i_sext;
		if (insn_lui) begin
			d_we = 1'b1;
			d_imm_val = {imm_u, 12'b000000000000};
		end
		else if ((((((((((((((((((insn_addi | insn_slti) | insn_sltiu) | insn_xori) | insn_ori) | insn_andi) | insn_slli) | insn_srli) | insn_srai) | insn_add) | insn_sub) | insn_sll) | insn_slt) | insn_sltu) | insn_xor) | insn_srl) | insn_sra) | insn_or) | insn_and)
			d_we = 1'b1;
	end
	function automatic [31:0] sv2v_cast_32;
		input reg [31:0] inp;
		sv2v_cast_32 = inp;
	endfunction
	always @(*) begin
		if (_sv2v_0)
			;
		if (flush_decode)
			execute_next = 240'h000000000000000000000008000000000000000000000000000000000000;
		else
			execute_next = {sv2v_cast_32(decode_state[95-:32]), sv2v_cast_32(decode_state[63-:32]), sv2v_cast_32(decode_state[31-:32]), d_rs1_data_bypassed, d_rs2_data_bypassed, d_imm_val, insn_rd, insn_rs1, insn_rs2, d_we, 32'd0};
	end
	always @(posedge clk)
		if (rst)
			execute_state <= 240'h000000000000000000000004000000000000000000000000000000000000;
		else
			execute_state <= execute_next;
	reg [165:0] memory_next;
	function automatic [4:0] sv2v_cast_5;
		input reg [4:0] inp;
		sv2v_cast_5 = inp;
	endfunction
	always @(*) begin
		if (_sv2v_0)
			;
		memory_next = {sv2v_cast_32(execute_state[239-:32]), sv2v_cast_32(execute_state[207-:32]), sv2v_cast_32(execute_state[175-:32]), e_rd_data, sv2v_cast_5(execute_state[47-:5]), execute_state[32], e_rd_data};
	end
	always @(posedge clk)
		if (rst)
			memory_state <= 166'h000000000000000000000001000000000000000000;
		else
			memory_state <= memory_next;
	assign addr_to_dmem = 32'd0;
	assign store_data_to_dmem = 32'd0;
	assign store_we_to_dmem = 4'b0000;
	reg [133:0] writeback_next;
	always @(*) begin
		if (_sv2v_0)
			;
		writeback_next = {sv2v_cast_32(memory_state[165-:32]), sv2v_cast_32(memory_state[133-:32]), sv2v_cast_32(memory_state[101-:32]), sv2v_cast_5(memory_state[37-:5]), memory_state[32], sv2v_cast_32(memory_state[31-:32])};
	end
	always @(posedge clk)
		if (rst)
			writeback_state <= 134'h0000000000000000000000010000000000;
		else
			writeback_state <= writeback_next;
	wire w_insn_ecall = (writeback_state[76:70] == OpcodeEnviron) && (writeback_state[101:77] == 25'd0);
	assign halt = w_insn_ecall;
	assign trace_completed_pc = writeback_state[133-:32];
	assign trace_completed_insn = writeback_state[101-:32];
	assign trace_completed_cycle_status = writeback_state[69-:32];
	initial _sv2v_0 = 0;
endmodule
module MemorySingleCycle (
	rst,
	clk,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem
);
	reg _sv2v_0;
	parameter signed [31:0] NUM_WORDS = 512;
	input wire rst;
	input wire clk;
	input wire [31:0] pc_to_imem;
	output wire [31:0] insn_from_imem;
	input wire [31:0] addr_to_dmem;
	output reg [31:0] load_data_from_dmem;
	input wire [31:0] store_data_to_dmem;
	input wire [3:0] store_we_to_dmem;
	reg [31:0] mem_array [0:NUM_WORDS - 1];
	initial $readmemh("mem_initial_contents.hex", mem_array);
	always @(*)
		if (_sv2v_0)
			;
	localparam signed [31:0] AddrMsb = $clog2(NUM_WORDS) + 1;
	localparam signed [31:0] AddrLsb = 2;
	assign insn_from_imem = mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
	always @(negedge clk)
		if (rst)
			;
		else begin
			if (store_we_to_dmem[0])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
			if (store_we_to_dmem[1])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
			if (store_we_to_dmem[2])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
			if (store_we_to_dmem[3])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
			load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
		end
	initial _sv2v_0 = 0;
endmodule
`default_nettype none
`default_nettype none
module SystemResourceCheck (
	external_clk_25MHz,
	btn,
	led
);
	input wire external_clk_25MHz;
	input wire [6:0] btn;
	output wire [7:0] led;
	wire clk_proc;
	wire clk_locked;
	MyClockGen clock_gen(
		.input_clk_25MHz(external_clk_25MHz),
		.clk_proc(clk_proc),
		.locked(clk_locked)
	);
	wire [31:0] pc_to_imem;
	wire [31:0] insn_from_imem;
	wire [31:0] mem_data_addr;
	wire [31:0] mem_data_loaded_value;
	wire [31:0] mem_data_to_write;
	wire [3:0] mem_data_we;
	wire [31:0] trace_writeback_pc;
	wire [31:0] trace_writeback_insn;
	wire [31:0] trace_writeback_cycle_status;
	MemorySingleCycle #(.NUM_WORDS(128)) memory(
		.rst(!clk_locked),
		.clk(clk_proc),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.load_data_from_dmem(mem_data_loaded_value),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we)
	);
	DatapathPipelined datapath(
		.clk(clk_proc),
		.rst(!clk_locked),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we),
		.load_data_from_dmem(mem_data_loaded_value),
		.halt(led[0]),
		.trace_completed_pc(trace_writeback_pc),
		.trace_completed_insn(trace_writeback_insn),
		.trace_completed_cycle_status(trace_writeback_cycle_status)
	);
endmodule