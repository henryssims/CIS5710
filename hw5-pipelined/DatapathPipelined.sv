`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`ifndef DIVIDER_STAGES
`define DIVIDER_STAGES 8
`endif

`ifndef SYNTHESIS
`include "../hw3-singlecycle/RvDisassembler.sv"
`endif
`include "../hw2b-cla/CarryLookaheadAdder.sv"
`include "../hw4-multicycle/DividerUnsignedPipelined.sv"
`include "../hw3-singlecycle/cycle_status.sv"

module Disasm #(
    byte PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
`ifndef SYNTHESIS
  // this code is only for simulation, not synthesis
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic. Also,
  // string needs to be reversed to render correctly.
  genvar i;
  for (i = 3; i < 32; i = i + 1) begin : gen_disasm
    assign disasm[((i+1-3)*8)-1-:8] = disasm_string[31-i];
  end
  assign disasm[255-:8] = PREFIX;
  assign disasm[247-:8] = ":";
  assign disasm[239-:8] = " ";
`endif
endmodule

module RegFile (
    input logic [4:0] rd,
    input logic [`REG_SIZE] rd_data,
    input logic [4:0] rs1,
    output logic [`REG_SIZE] rs1_data,
    input logic [4:0] rs2,
    output logic [`REG_SIZE] rs2_data,

    input logic clk,
    input logic we,
    input logic rst
);
  localparam int NumRegs = 32;
  logic [`REG_SIZE] regs[NumRegs];

  assign regs[0] = 32'd0;
  assign rs1_data = regs[rs1];
  assign rs2_data = regs[rs2];
  genvar i;
  for (i = 1; i < 32; i = i + 1) begin
    always_ff @(posedge clk) begin
      if (rst) begin
        regs[i] <= 32'd0;
      end else begin
        if (we && rd == 5'(i)) begin
          regs[i] <= rd_data;
        end
      end
    end
  end
endmodule

/** state at the start of Decode stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
} stage_decode_t;

/** state at the start of Execute stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic [`REG_SIZE] rs1_data;
  logic [`REG_SIZE] rs2_data;
  logic [`REG_SIZE] imm;
  logic [4:0] rd;
  logic [4:0] rs1;
  logic [4:0] rs2;
  logic we;
  logic [`REG_SIZE] rd_data;
} stage_execute_t;

/** state at the start of Memory stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic [`REG_SIZE] alu_result;
  logic [4:0] rd;
  logic we;
  logic [`REG_SIZE] rd_data;
} stage_memory_t;

/** state at the start of Writeback stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic [4:0] rd;
  logic we;
  logic [`REG_SIZE] rd_data;
} stage_writeback_t;

module DatapathPipelined (
    input wire clk,
    input wire rst,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`INSN_SIZE] insn_from_imem,
    // dmem is read/write
    output logic [`REG_SIZE] addr_to_dmem,
    input wire [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_completed_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_completed_insn,
    // The status of the insn (or stall) currently in Writeback. See the cycle_status.sv file for valid values.
    output cycle_status_e trace_completed_cycle_status
);

  // opcodes - see section 19 of RiscV spec
  /* verilator lint_off UNUSEDPARAM */
  localparam bit [`OPCODE_SIZE] OpcodeLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpcodeMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpcodeJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpcodeRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpcodeAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpcodeLui = 7'b01_101_11;
  /* verilator lint_on UNUSEDPARAM */

  // cycle counter, not really part of any stage but useful for orienting within GtkWave
  // do not rename this as the testbench uses this value
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  wire [6:0] insn_funct7;
  wire [4:0] insn_rs2, insn_rs1;
  wire [2:0] insn_funct3;
  wire [4:0] insn_rd;
  wire [6:0] insn_opcode;
  wire [11:0] imm_i;
  wire [4:0] imm_shamt;
  wire [11:0] imm_s;
  wire [12:0] imm_b;

  wire [19:0] imm_u;
  wire [`REG_SIZE] imm_i_sext, imm_s_sext, imm_b_sext;

  logic branch_taken;
  logic [`REG_SIZE] branch_target_actual;

  /***************/
  /* FETCH STAGE */
  /***************/

  logic [`REG_SIZE] f_pc_current;
  logic [`REG_SIZE] f_pc_sent;
  wire [`REG_SIZE] f_insn;
  cycle_status_e f_cycle_status;

  logic [`REG_SIZE] f_pc_next;
  always_comb begin
    f_pc_next = f_pc_current + 4;
    if (branch_taken)
      f_pc_next = branch_target_actual;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 32'd0;
      f_pc_sent <= 32'd0;
      f_cycle_status <= CYCLE_NO_STALL;
    end else begin
      f_pc_sent <= f_pc_current;
      f_pc_current <= f_pc_next;
      f_cycle_status <= CYCLE_NO_STALL;
    end
  end
  assign pc_to_imem = f_pc_current;
  assign f_insn = insn_from_imem;

  wire [255:0] f_disasm;
  Disasm #(.PREFIX("F")) disasm_0fetch (.insn(f_insn), .disasm(f_disasm));

  /****************/
  /* DECODE STAGE */
  /****************/

  stage_decode_t decode_state;
  stage_decode_t decode_next;

  logic [`REG_SIZE] d_rs1_data, d_rs2_data;
  logic [`REG_SIZE] d_rs1_data_bypassed, d_rs2_data_bypassed;

  always_comb begin
    d_rs1_data_bypassed = d_rs1_data;
    d_rs2_data_bypassed = d_rs2_data;
    if (writeback_state.we && writeback_state.rd != 5'd0) begin
      if (writeback_state.rd == insn_rs1) d_rs1_data_bypassed = writeback_state.rd_data;
      if (writeback_state.rd == insn_rs2) d_rs2_data_bypassed = writeback_state.rd_data;
    end
  end

  assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = decode_state.insn;
  assign imm_u = decode_state.insn[31:12];
  assign imm_i = decode_state.insn[31:20];
  assign imm_shamt = decode_state.insn[24:20];
  assign imm_s = {insn_funct7, insn_rd};
  assign {imm_b[12], imm_b[10:5]} = insn_funct7;
  assign {imm_b[4:1], imm_b[11]} = insn_rd;
  assign imm_b[0] = 1'b0;

  assign imm_i_sext = {{20{imm_i[11]}}, imm_i};
  assign imm_s_sext = {{20{imm_s[11]}}, imm_s};
  assign imm_b_sext = {{19{imm_b[12]}}, imm_b};

  RegFile rf (
    .clk(clk),
    .rst(rst),
    .we(writeback_state.we),
    .rd(writeback_state.rd),
    .rd_data(writeback_state.rd_data),
    .rs1(insn_rs1),
    .rs2(insn_rs2),
    .rs1_data(d_rs1_data),
    .rs2_data(d_rs2_data));

  logic flush_decode;
  assign flush_decode = branch_taken;

  always_comb begin
    if (flush_decode) begin
      decode_next = '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_TAKEN_BRANCH
      };
    end else begin
      decode_next = '{
        pc: f_pc_current,
        insn: f_insn,
        cycle_status: f_cycle_status
      };
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      decode_state <= '{pc: 0, insn: 0, cycle_status: CYCLE_RESET};
    end else begin
      decode_state <= decode_next;
    end
  end

  wire [255:0] d_disasm;
  Disasm #(.PREFIX("D")) disasm_1decode (.insn(decode_state.insn), .disasm(d_disasm));

  /******************/
  /* EXECUTE STAGE */
  /******************/

  stage_execute_t execute_state;
  stage_execute_t execute_next;

  wire insn_lui   = (insn_opcode == OpcodeLui);
  wire insn_beq   = (insn_opcode == OpcodeBranch) && (decode_state.insn[14:12] == 3'b000);
  wire insn_bne   = (insn_opcode == OpcodeBranch) && (decode_state.insn[14:12] == 3'b001);
  wire insn_blt   = (insn_opcode == OpcodeBranch) && (decode_state.insn[14:12] == 3'b100);
  wire insn_bge   = (insn_opcode == OpcodeBranch) && (decode_state.insn[14:12] == 3'b101);
  wire insn_bltu  = (insn_opcode == OpcodeBranch) && (decode_state.insn[14:12] == 3'b110);
  wire insn_bgeu  = (insn_opcode == OpcodeBranch) && (decode_state.insn[14:12] == 3'b111);

  wire insn_addi  = (insn_opcode == OpcodeRegImm) && (decode_state.insn[14:12] == 3'b000);
  wire insn_slti  = (insn_opcode == OpcodeRegImm) && (decode_state.insn[14:12] == 3'b010);
  wire insn_sltiu = (insn_opcode == OpcodeRegImm) && (decode_state.insn[14:12] == 3'b011);
  wire insn_xori  = (insn_opcode == OpcodeRegImm) && (decode_state.insn[14:12] == 3'b100);
  wire insn_ori   = (insn_opcode == OpcodeRegImm) && (decode_state.insn[14:12] == 3'b110);
  wire insn_andi  = (insn_opcode == OpcodeRegImm) && (decode_state.insn[14:12] == 3'b111);
  wire insn_slli  = (insn_opcode == OpcodeRegImm) && (decode_state.insn[14:12] == 3'b001) && (decode_state.insn[31:25] == 7'd0);
  wire insn_srli  = (insn_opcode == OpcodeRegImm) && (decode_state.insn[14:12] == 3'b101) && (decode_state.insn[31:25] == 7'd0);
  wire insn_srai  = (insn_opcode == OpcodeRegImm) && (decode_state.insn[14:12] == 3'b101) && (decode_state.insn[31:25] == 7'b0100000);

  wire insn_add   = (insn_opcode == OpcodeRegReg) && (decode_state.insn[14:12] == 3'b000) && (decode_state.insn[31:25] == 7'd0);
  wire insn_sub   = (insn_opcode == OpcodeRegReg) && (decode_state.insn[14:12] == 3'b000) && (decode_state.insn[31:25] == 7'b0100000);
  wire insn_sll   = (insn_opcode == OpcodeRegReg) && (decode_state.insn[14:12] == 3'b001) && (decode_state.insn[31:25] == 7'd0);
  wire insn_slt   = (insn_opcode == OpcodeRegReg) && (decode_state.insn[14:12] == 3'b010) && (decode_state.insn[31:25] == 7'd0);
  wire insn_sltu  = (insn_opcode == OpcodeRegReg) && (decode_state.insn[14:12] == 3'b011) && (decode_state.insn[31:25] == 7'd0);
  wire insn_xor   = (insn_opcode == OpcodeRegReg) && (decode_state.insn[14:12] == 3'b100) && (decode_state.insn[31:25] == 7'd0);
  wire insn_srl   = (insn_opcode == OpcodeRegReg) && (decode_state.insn[14:12] == 3'b101) && (decode_state.insn[31:25] == 7'd0);
  wire insn_sra   = (insn_opcode == OpcodeRegReg) && (decode_state.insn[14:12] == 3'b101) && (decode_state.insn[31:25] == 7'b0100000);
  wire insn_or    = (insn_opcode == OpcodeRegReg) && (decode_state.insn[14:12] == 3'b110) && (decode_state.insn[31:25] == 7'd0);
  wire insn_and   = (insn_opcode == OpcodeRegReg) && (decode_state.insn[14:12] == 3'b111) && (decode_state.insn[31:25] == 7'd0);

  wire insn_ecall = (insn_opcode == OpcodeEnviron) && (decode_state.insn[31:7] == 25'd0);

  stage_memory_t memory_state;
  stage_writeback_t writeback_state;

  logic [`REG_SIZE] e_rs1_val, e_rs2_val;
  always_comb begin
    e_rs1_val = execute_state.rs1_data;
    e_rs2_val = execute_state.rs2_data;
    if (writeback_state.we && writeback_state.rd != 5'd0) begin
      if (writeback_state.rd == execute_state.rs1) e_rs1_val = writeback_state.rd_data;
      if (writeback_state.rd == execute_state.rs2) e_rs2_val = writeback_state.rd_data;
    end
    if (memory_state.we && memory_state.rd != 5'd0) begin
      if (memory_state.rd == execute_state.rs1) e_rs1_val = memory_state.alu_result;
      if (memory_state.rd == execute_state.rs2) e_rs2_val = memory_state.alu_result;
    end
  end

  wire e_insn_lui   = (execute_state.insn[6:0] == OpcodeLui);
  wire e_insn_addi  = (execute_state.insn[6:0] == OpcodeRegImm) && (execute_state.insn[14:12] == 3'b000);
  wire e_insn_slti  = (execute_state.insn[6:0] == OpcodeRegImm) && (execute_state.insn[14:12] == 3'b010);
  wire e_insn_sltiu = (execute_state.insn[6:0] == OpcodeRegImm) && (execute_state.insn[14:12] == 3'b011);
  wire e_insn_xori  = (execute_state.insn[6:0] == OpcodeRegImm) && (execute_state.insn[14:12] == 3'b100);
  wire e_insn_ori   = (execute_state.insn[6:0] == OpcodeRegImm) && (execute_state.insn[14:12] == 3'b110);
  wire e_insn_andi  = (execute_state.insn[6:0] == OpcodeRegImm) && (execute_state.insn[14:12] == 3'b111);
  wire e_insn_slli  = (execute_state.insn[6:0] == OpcodeRegImm) && (execute_state.insn[14:12] == 3'b001) && (execute_state.insn[31:25] == 7'd0);
  wire e_insn_srli  = (execute_state.insn[6:0] == OpcodeRegImm) && (execute_state.insn[14:12] == 3'b101) && (execute_state.insn[31:25] == 7'd0);
  wire e_insn_srai  = (execute_state.insn[6:0] == OpcodeRegImm) && (execute_state.insn[14:12] == 3'b101) && (execute_state.insn[31:25] == 7'b0100000);
  wire e_insn_add   = (execute_state.insn[6:0] == OpcodeRegReg) && (execute_state.insn[14:12] == 3'b000) && (execute_state.insn[31:25] == 7'd0);
  wire e_insn_sub   = (execute_state.insn[6:0] == OpcodeRegReg) && (execute_state.insn[14:12] == 3'b000) && (execute_state.insn[31:25] == 7'b0100000);
  wire e_insn_sll   = (execute_state.insn[6:0] == OpcodeRegReg) && (execute_state.insn[14:12] == 3'b001) && (execute_state.insn[31:25] == 7'd0);
  wire e_insn_slt   = (execute_state.insn[6:0] == OpcodeRegReg) && (execute_state.insn[14:12] == 3'b010) && (execute_state.insn[31:25] == 7'd0);
  wire e_insn_sltu  = (execute_state.insn[6:0] == OpcodeRegReg) && (execute_state.insn[14:12] == 3'b011) && (execute_state.insn[31:25] == 7'd0);
  wire e_insn_xor   = (execute_state.insn[6:0] == OpcodeRegReg) && (execute_state.insn[14:12] == 3'b100) && (execute_state.insn[31:25] == 7'd0);
  wire e_insn_srl   = (execute_state.insn[6:0] == OpcodeRegReg) && (execute_state.insn[14:12] == 3'b101) && (execute_state.insn[31:25] == 7'd0);
  wire e_insn_sra   = (execute_state.insn[6:0] == OpcodeRegReg) && (execute_state.insn[14:12] == 3'b101) && (execute_state.insn[31:25] == 7'b0100000);
  wire e_insn_or    = (execute_state.insn[6:0] == OpcodeRegReg) && (execute_state.insn[14:12] == 3'b110) && (execute_state.insn[31:25] == 7'd0);
  wire e_insn_and   = (execute_state.insn[6:0] == OpcodeRegReg) && (execute_state.insn[14:12] == 3'b111) && (execute_state.insn[31:25] == 7'd0);

  logic [`REG_SIZE] alu_a, alu_b, alu_result;
  logic alu_cin;
  CarryLookaheadAdder alu_adder (.a(alu_a), .b(alu_b), .cin(alu_cin), .sum(alu_result));

  logic [`REG_SIZE] e_rd_data;
  always_comb begin
    alu_a = 32'd0;
    alu_b = 32'd0;
    alu_cin = 1'b0;
    e_rd_data = 32'd0;
    if (e_insn_lui)
      e_rd_data = execute_state.imm;
    else if (e_insn_addi) begin
      alu_a = e_rs1_val;
      alu_b = execute_state.imm;
      e_rd_data = alu_result;
    end else if (e_insn_slti)
      e_rd_data = ($signed(e_rs1_val) < $signed(execute_state.imm)) ? 32'b1 : 32'b0;
    else if (e_insn_sltiu)
      e_rd_data = ($unsigned(e_rs1_val) < $unsigned(execute_state.imm)) ? 32'b1 : 32'b0;
    else if (e_insn_xori)
      e_rd_data = e_rs1_val ^ execute_state.imm;
    else if (e_insn_ori)
      e_rd_data = e_rs1_val | execute_state.imm;
    else if (e_insn_andi)
      e_rd_data = e_rs1_val & execute_state.imm;
    else if (e_insn_slli)
      e_rd_data = e_rs1_val << execute_state.imm[4:0];
    else if (e_insn_srli)
      e_rd_data = e_rs1_val >> execute_state.imm[4:0];
    else if (e_insn_srai)
      e_rd_data = $signed(e_rs1_val) >>> execute_state.imm[4:0];
    else if (e_insn_add) begin
      alu_a = e_rs1_val;
      alu_b = e_rs2_val;
      e_rd_data = alu_result;
    end else if (e_insn_sub) begin
      alu_a = e_rs1_val;
      alu_b = ~e_rs2_val;
      alu_cin = 1'b1;
      e_rd_data = alu_result;
    end else if (e_insn_sll)
      e_rd_data = e_rs1_val << e_rs2_val[4:0];
    else if (e_insn_slt)
      e_rd_data = ($signed(e_rs1_val) < $signed(e_rs2_val)) ? 32'b1 : 32'b0;
    else if (e_insn_sltu)
      e_rd_data = ($unsigned(e_rs1_val) < $unsigned(e_rs2_val)) ? 32'b1 : 32'b0;
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

  wire e_insn_beq  = (execute_state.insn[6:0] == OpcodeBranch) && (execute_state.insn[14:12] == 3'b000);
  wire e_insn_bne  = (execute_state.insn[6:0] == OpcodeBranch) && (execute_state.insn[14:12] == 3'b001);
  wire e_insn_blt  = (execute_state.insn[6:0] == OpcodeBranch) && (execute_state.insn[14:12] == 3'b100);
  wire e_insn_bge  = (execute_state.insn[6:0] == OpcodeBranch) && (execute_state.insn[14:12] == 3'b101);
  wire e_insn_bltu = (execute_state.insn[6:0] == OpcodeBranch) && (execute_state.insn[14:12] == 3'b110);
  wire e_insn_bgeu = (execute_state.insn[6:0] == OpcodeBranch) && (execute_state.insn[14:12] == 3'b111);

  wire [12:0] e_imm_b;
  assign {e_imm_b[12], e_imm_b[10:5]} = execute_state.insn[31:25];
  assign {e_imm_b[4:1], e_imm_b[11]} = execute_state.insn[11:7];
  assign e_imm_b[0] = 1'b0;

  wire [`REG_SIZE] e_branch_target;
  CarryLookaheadAdder e_branch_adder (
    .a(execute_state.pc),
    .b({{19{e_imm_b[12]}}, e_imm_b[12:0]}),
    .cin(1'b0),
    .sum(e_branch_target));

  always_comb begin
    branch_taken = 1'b0;
    branch_target_actual = e_branch_target;
    if (e_insn_beq)  branch_taken = (e_rs1_val == e_rs2_val);
    else if (e_insn_bne)  branch_taken = (e_rs1_val != e_rs2_val);
    else if (e_insn_blt)  branch_taken = ($signed(e_rs1_val) < $signed(e_rs2_val));
    else if (e_insn_bge)  branch_taken = ($signed(e_rs1_val) >= $signed(e_rs2_val));
    else if (e_insn_bltu) branch_taken = ($unsigned(e_rs1_val) < $unsigned(e_rs2_val));
    else if (e_insn_bgeu) branch_taken = ($unsigned(e_rs1_val) >= $unsigned(e_rs2_val));
  end

  logic d_we;
  logic [`REG_SIZE] d_imm_val;
  always_comb begin
    d_we = 1'b0;
    d_imm_val = imm_i_sext;
    if (insn_lui) begin
      d_we = 1'b1;
      d_imm_val = {imm_u, 12'b0};
    end else if (insn_addi | insn_slti | insn_sltiu | insn_xori | insn_ori | insn_andi |
                insn_slli | insn_srli | insn_srai | insn_add | insn_sub | insn_sll |
                insn_slt | insn_sltu | insn_xor | insn_srl | insn_sra | insn_or | insn_and)
      d_we = 1'b1;
  end

  always_comb begin
    if (flush_decode) begin
      execute_next = '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_TAKEN_BRANCH,
        rs1_data: 0, rs2_data: 0, imm: 0, rd: 0, rs1: 0, rs2: 0, we: 0, rd_data: 0
      };
    end else begin
      execute_next = '{
        pc: decode_state.pc,
        insn: decode_state.insn,
        cycle_status: decode_state.cycle_status,
        rs1_data: d_rs1_data_bypassed,
        rs2_data: d_rs2_data_bypassed,
        imm: d_imm_val,
        rd: insn_rd,
        rs1: insn_rs1,
        rs2: insn_rs2,
        we: d_we,
        rd_data: 32'd0
      };
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      execute_state <= '{
        pc: 0, insn: 0, cycle_status: CYCLE_RESET,
        rs1_data: 0, rs2_data: 0, imm: 0, rd: 0, rs1: 0, rs2: 0, we: 0, rd_data: 0
      };
    end else begin
      execute_state <= execute_next;
    end
  end

  /******************/
  /* MEMORY STAGE   */
  /******************/

  stage_memory_t memory_next;

  always_comb begin
    memory_next = '{
      pc: execute_state.pc,
      insn: execute_state.insn,
      cycle_status: execute_state.cycle_status,
      alu_result: e_rd_data,
      rd: execute_state.rd,
      we: execute_state.we,
      rd_data: e_rd_data
    };
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      memory_state <= '{
        pc: 0, insn: 0, cycle_status: CYCLE_RESET,
        alu_result: 0, rd: 0, we: 0, rd_data: 0
      };
    end else begin
      memory_state <= memory_next;
    end
  end

  assign addr_to_dmem = 32'd0;
  assign store_data_to_dmem = 32'd0;
  assign store_we_to_dmem = 4'b0;

  /*******************/
  /* WRITEBACK STAGE */
  /*******************/

  stage_writeback_t writeback_next;

  always_comb begin
    writeback_next = '{
      pc: memory_state.pc,
      insn: memory_state.insn,
      cycle_status: memory_state.cycle_status,
      rd: memory_state.rd,
      we: memory_state.we,
      rd_data: memory_state.rd_data
    };
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      writeback_state <= '{
        pc: 0, insn: 0, cycle_status: CYCLE_RESET,
        rd: 0, we: 0, rd_data: 0
      };
    end else begin
      writeback_state <= writeback_next;
    end
  end

  wire w_insn_ecall = (writeback_state.insn[6:0] == OpcodeEnviron) && (writeback_state.insn[31:7] == 25'd0);
  assign halt = w_insn_ecall;

  assign trace_completed_pc = writeback_state.pc;
  assign trace_completed_insn = writeback_state.insn;
  assign trace_completed_cycle_status = writeback_state.cycle_status;

endmodule

module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. The memory reads/writes on @(negedge clk)
    input wire clk,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] pc_to_imem,

    // the value at memory location pc_to_imem
    output logic [`REG_SIZE] insn_from_imem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] addr_to_dmem,

    // the value at memory location addr_to_dmem
    output logic [`REG_SIZE] load_data_from_dmem,

    // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
    input wire [`REG_SIZE] store_data_to_dmem,

    // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
    // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
    input wire [3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  logic [`REG_SIZE] mem_array[NUM_WORDS];

`ifdef SYNTHESIS
  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end
`endif

  always_comb begin
    // memory addresses should always be 4B-aligned
    assert (pc_to_imem[1:0] == 2'b00);
    assert (addr_to_dmem[1:0] == 2'b00);
  end

  localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam int AddrLsb = 2;

  // Combinational read: instruction available same cycle as address (matches reference trace timing)
  assign insn_from_imem = mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      if (store_we_to_dmem[0]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
      end
      if (store_we_to_dmem[1]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
      end
      if (store_we_to_dmem[2]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
      end
      if (store_we_to_dmem[3]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
      end
      // dmem is "read-first": read returns value before the write
      load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
    end
  end
endmodule

/* This design has just one clock for both processor and memory. */
module Processor (
    input  wire  clk,
    input  wire  rst,
    output logic halt,
    output wire [`REG_SIZE] trace_completed_pc,
    output wire [`INSN_SIZE] trace_completed_insn,
    output cycle_status_e trace_completed_cycle_status
);

  wire [`INSN_SIZE] insn_from_imem;
  wire [`REG_SIZE] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
      .rst                (rst),
      .clk                (clk),
      // imem is read-only
      .pc_to_imem         (pc_to_imem),
      .insn_from_imem     (insn_from_imem),
      // dmem is read-write
      .addr_to_dmem       (mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem   (mem_data_we)
  );

  DatapathPipelined datapath (
      .clk(clk),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt),
      .trace_completed_pc(trace_completed_pc),
      .trace_completed_insn(trace_completed_insn),
      .trace_completed_cycle_status(trace_completed_cycle_status)
  );

endmodule
