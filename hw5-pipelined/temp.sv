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
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn);
  end
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
  logic is_load;
  logic is_store;
  logic [2:0] funct3;
  logic [`REG_SIZE] store_val;
  logic div_slow;
  logic div_signed;
  logic div_is_rem;
  logic rs1_neg;
  logic rs2_neg;
  logic [`REG_SIZE] rs1_orig;
  logic [4:0] rs2_shadow;
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
    output logic [`REG_SIZE] addr_to_dmem,
    input wire [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem,

    output logic halt,

    output logic [`REG_SIZE] trace_completed_pc,
    output logic [`INSN_SIZE] trace_completed_insn,
    output cycle_status_e trace_completed_cycle_status
);

  localparam bit [`OPCODE_SIZE] OpcodeLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeJalr = 7'b11_001_11;
  /* verilator lint_off UNUSEDPARAM */
  localparam bit [`OPCODE_SIZE] OpcodeMiscMem = 7'b00_011_11;
  /* verilator lint_on UNUSEDPARAM */
  localparam bit [`OPCODE_SIZE] OpcodeJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpcodeRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpcodeAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpcodeLui = 7'b01_101_11;

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
  wire [20:0] imm_j;
  wire [`REG_SIZE] imm_j_sext;

  logic branch_taken;
  logic [`REG_SIZE] branch_target_actual;

  /***************/
  /* FETCH STAGE */
  /***************/

  logic [`REG_SIZE] f_pc_current;
  logic [`REG_SIZE] f_pc_sent;
  wire [`REG_SIZE] f_insn;
  cycle_status_e f_cycle_status;

  logic stall_mem_div;
  logic stall_front;

  logic [`REG_SIZE] f_pc_next;
  always_comb begin
    f_pc_next = f_pc_current + 4;
    if (branch_taken)
      f_pc_next = branch_target_actual;
    else if (stall_front)
      f_pc_next = f_pc_current;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 32'd0;
      f_pc_sent <= 32'd0;
      f_cycle_status <= CYCLE_NO_STALL;
    end else begin
      if (!stall_front) begin
        f_pc_sent <= f_pc_current;
        f_pc_current <= f_pc_next;
      end
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

  stage_memory_t memory_state;
  stage_writeback_t writeback_state;

  assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = decode_state.insn;
  assign imm_u = decode_state.insn[31:12];
  assign imm_i = decode_state.insn[31:20];
  assign imm_shamt = decode_state.insn[24:20];
  assign imm_s = {insn_funct7, insn_rd};
  assign {imm_b[12], imm_b[10:5]} = insn_funct7;
  assign {imm_b[4:1], imm_b[11]} = insn_rd;
  assign imm_b[0] = 1'b0;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {decode_state.insn[31:12], 1'b0};

  assign imm_i_sext = {{20{imm_i[11]}}, imm_i};
  assign imm_s_sext = {{20{imm_s[11]}}, imm_s};
  assign imm_b_sext = {{19{imm_b[12]}}, imm_b};
  assign imm_j_sext = {{11{imm_j[20]}}, imm_j};

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

  wire [6:0] d_opcode = decode_state.insn[6:0];
  wire d_reads_rs1 = (d_opcode == OpcodeLoad) || (d_opcode == OpcodeStore) || (d_opcode == OpcodeBranch) ||
      (d_opcode == OpcodeJalr) || (d_opcode == OpcodeRegImm) || (d_opcode == OpcodeRegReg) ||
      (d_opcode == OpcodeAuipc);
  wire d_reads_rs2 = (d_opcode == OpcodeBranch) || (d_opcode == OpcodeStore) || (d_opcode == OpcodeRegReg);

  logic [7:0] div_inflight_valid;
  logic [4:0] div_inflight_rd[8];
  // Registered before stall_front uses stall_div_overlap (see divider block).
  logic [2:0] div_hold_cnt;

  logic stall_div_raw;
  always_comb begin
    stall_div_raw = 1'b0;
    if (d_reads_rs1 && insn_rs1 != 5'd0) begin
      for (int k = 0; k < 7; k++) begin
        if (div_inflight_valid[k] && div_inflight_rd[k] == insn_rs1)
          stall_div_raw = 1'b1;
      end
    end
    if (d_reads_rs2 && insn_rs2 != 5'd0) begin
      for (int k = 0; k < 7; k++) begin
        if (div_inflight_valid[k] && div_inflight_rd[k] == insn_rs2)
          stall_div_raw = 1'b1;
      end
    end
  end

  stage_execute_t execute_state;

  wire exe_is_load = (execute_state.insn[6:0] == OpcodeLoad);
  wire [4:0] exe_rd = execute_state.rd;
  wire stall_load_use = exe_is_load && exe_rd != 5'd0 &&
      ((d_reads_rs1 && insn_rs1 == exe_rd) || (d_reads_rs2 && insn_rs2 == exe_rd));

  wire stall_div_overlap;
  assign stall_front = stall_load_use || stall_div_raw || stall_mem_div || stall_div_overlap;

  cycle_status_e stall_status;
  always_comb begin
    stall_status = CYCLE_NO_STALL;
    if (stall_load_use)
      stall_status = CYCLE_LOAD2USE;
    else if (stall_div_raw || stall_div_overlap)
      stall_status = CYCLE_DIV;
  end

  wire stall_decode_bubble_execute = stall_load_use || stall_div_raw || stall_div_overlap;

  always_comb begin
    if (flush_decode) begin
      decode_next = '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_TAKEN_BRANCH
      };
    end else if (stall_front) begin
      decode_next = decode_state;
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

  stage_execute_t execute_next;

  wire insn_lui = (insn_opcode == OpcodeLui);
  wire insn_auipc = (insn_opcode == OpcodeAuipc);
  wire insn_jal = (insn_opcode == OpcodeJal);
  wire insn_jalr = (insn_opcode == OpcodeJalr);

  wire insn_beq = (insn_opcode == OpcodeBranch) && (insn_funct3 == 3'b000);
  wire insn_bne = (insn_opcode == OpcodeBranch) && (insn_funct3 == 3'b001);
  wire insn_blt = (insn_opcode == OpcodeBranch) && (insn_funct3 == 3'b100);
  wire insn_bge = (insn_opcode == OpcodeBranch) && (insn_funct3 == 3'b101);
  wire insn_bltu = (insn_opcode == OpcodeBranch) && (insn_funct3 == 3'b110);
  wire insn_bgeu = (insn_opcode == OpcodeBranch) && (insn_funct3 == 3'b111);

  wire insn_lb = (insn_opcode == OpcodeLoad) && (insn_funct3 == 3'b000);
  wire insn_lh = (insn_opcode == OpcodeLoad) && (insn_funct3 == 3'b001);
  wire insn_lw = (insn_opcode == OpcodeLoad) && (insn_funct3 == 3'b010);
  wire insn_lbu = (insn_opcode == OpcodeLoad) && (insn_funct3 == 3'b100);
  wire insn_lhu = (insn_opcode == OpcodeLoad) && (insn_funct3 == 3'b101);

  wire insn_sb = (insn_opcode == OpcodeStore) && (insn_funct3 == 3'b000);
  wire insn_sh = (insn_opcode == OpcodeStore) && (insn_funct3 == 3'b001);
  wire insn_sw = (insn_opcode == OpcodeStore) && (insn_funct3 == 3'b010);

  wire insn_addi = (insn_opcode == OpcodeRegImm) && (insn_funct3 == 3'b000);
  wire insn_slti = (insn_opcode == OpcodeRegImm) && (insn_funct3 == 3'b010);
  wire insn_sltiu = (insn_opcode == OpcodeRegImm) && (insn_funct3 == 3'b011);
  wire insn_xori = (insn_opcode == OpcodeRegImm) && (insn_funct3 == 3'b100);
  wire insn_ori = (insn_opcode == OpcodeRegImm) && (insn_funct3 == 3'b110);
  wire insn_andi = (insn_opcode == OpcodeRegImm) && (insn_funct3 == 3'b111);
  wire insn_slli = (insn_opcode == OpcodeRegImm) && (insn_funct3 == 3'b001) && (insn_funct7 == 7'd0);
  wire insn_srli = (insn_opcode == OpcodeRegImm) && (insn_funct3 == 3'b101) && (insn_funct7 == 7'd0);
  wire insn_srai = (insn_opcode == OpcodeRegImm) && (insn_funct3 == 3'b101) && (insn_funct7 == 7'b0100000);

  wire insn_add = (insn_opcode == OpcodeRegReg) && (insn_funct3 == 3'b000) && (insn_funct7 == 7'd0);
  wire insn_sub = (insn_opcode == OpcodeRegReg) && (insn_funct3 == 3'b000) && (insn_funct7 == 7'b0100000);
  wire insn_sll = (insn_opcode == OpcodeRegReg) && (insn_funct3 == 3'b001) && (insn_funct7 == 7'd0);
  wire insn_slt = (insn_opcode == OpcodeRegReg) && (insn_funct3 == 3'b010) && (insn_funct7 == 7'd0);
  wire insn_sltu = (insn_opcode == OpcodeRegReg) && (insn_funct3 == 3'b011) && (insn_funct7 == 7'd0);
  wire insn_xor = (insn_opcode == OpcodeRegReg) && (insn_funct3 == 3'b100) && (insn_funct7 == 7'd0);
  wire insn_srl = (insn_opcode == OpcodeRegReg) && (insn_funct3 == 3'b101) && (insn_funct7 == 7'd0);
  wire insn_sra = (insn_opcode == OpcodeRegReg) && (insn_funct3 == 3'b101) && (insn_funct7 == 7'b0100000);
  wire insn_or = (insn_opcode == OpcodeRegReg) && (insn_funct3 == 3'b110) && (insn_funct7 == 7'd0);
  wire insn_and = (insn_opcode == OpcodeRegReg) && (insn_funct3 == 3'b111) && (insn_funct7 == 7'd0);

  wire insn_mul = (insn_opcode == OpcodeRegReg) && (insn_funct7 == 7'd1) && (insn_funct3 == 3'b000);
  wire insn_mulh = (insn_opcode == OpcodeRegReg) && (insn_funct7 == 7'd1) && (insn_funct3 == 3'b001);
  wire insn_mulhsu = (insn_opcode == OpcodeRegReg) && (insn_funct7 == 7'd1) && (insn_funct3 == 3'b010);
  wire insn_mulhu = (insn_opcode == OpcodeRegReg) && (insn_funct7 == 7'd1) && (insn_funct3 == 3'b011);
  wire insn_div = (insn_opcode == OpcodeRegReg) && (insn_funct7 == 7'd1) && (insn_funct3 == 3'b100);
  wire insn_divu = (insn_opcode == OpcodeRegReg) && (insn_funct7 == 7'd1) && (insn_funct3 == 3'b101);
  wire insn_rem = (insn_opcode == OpcodeRegReg) && (insn_funct7 == 7'd1) && (insn_funct3 == 3'b110);
  wire insn_remu = (insn_opcode == OpcodeRegReg) && (insn_funct7 == 7'd1) && (insn_funct3 == 3'b111);

  wire insn_ecall = (insn_opcode == OpcodeEnviron) && (decode_state.insn[31:7] == 25'd0);

  logic [`REG_SIZE] e_rs1_val, e_rs2_val;
  always_comb begin
    e_rs1_val = execute_state.rs1_data;
    e_rs2_val = execute_state.rs2_data;
    if (writeback_state.we && writeback_state.rd != 5'd0) begin
      if (writeback_state.rd == execute_state.rs1) e_rs1_val = writeback_state.rd_data;
      if (writeback_state.rd == execute_state.rs2) e_rs2_val = writeback_state.rd_data;
    end
    if (memory_state.we && memory_state.rd != 5'd0) begin
      if (memory_state.rd == execute_state.rs1) begin
        if (memory_state.is_load)
          e_rs1_val = load_data_from_dmem;
        else
          e_rs1_val = memory_state.rd_data;
      end
      if (memory_state.rd == execute_state.rs2) begin
        if (memory_state.is_load)
          e_rs2_val = load_data_from_dmem;
        else
          e_rs2_val = memory_state.rd_data;
      end
    end
  end

  wire [31:0] e_insn = execute_state.insn;
  wire [6:0] e_opcode = e_insn[6:0];
  wire [2:0] e_f3 = e_insn[14:12];
  wire [6:0] e_f7 = e_insn[31:25];

  wire e_insn_lui = (e_opcode == OpcodeLui);
  wire e_insn_auipc = (e_opcode == OpcodeAuipc);
  wire e_insn_jal = (e_opcode == OpcodeJal);
  wire e_insn_jalr = (e_opcode == OpcodeJalr);

  wire e_insn_beq = (e_opcode == OpcodeBranch) && (e_f3 == 3'b000);
  wire e_insn_bne = (e_opcode == OpcodeBranch) && (e_f3 == 3'b001);
  wire e_insn_blt = (e_opcode == OpcodeBranch) && (e_f3 == 3'b100);
  wire e_insn_bge = (e_opcode == OpcodeBranch) && (e_f3 == 3'b101);
  wire e_insn_bltu = (e_opcode == OpcodeBranch) && (e_f3 == 3'b110);
  wire e_insn_bgeu = (e_opcode == OpcodeBranch) && (e_f3 == 3'b111);

  wire e_insn_lb = (e_opcode == OpcodeLoad) && (e_f3 == 3'b000);
  wire e_insn_lh = (e_opcode == OpcodeLoad) && (e_f3 == 3'b001);
  wire e_insn_lw = (e_opcode == OpcodeLoad) && (e_f3 == 3'b010);
  wire e_insn_lbu = (e_opcode == OpcodeLoad) && (e_f3 == 3'b100);
  wire e_insn_lhu = (e_opcode == OpcodeLoad) && (e_f3 == 3'b101);

  wire e_insn_sb = (e_opcode == OpcodeStore) && (e_f3 == 3'b000);
  wire e_insn_sh = (e_opcode == OpcodeStore) && (e_f3 == 3'b001);
  wire e_insn_sw = (e_opcode == OpcodeStore) && (e_f3 == 3'b010);

  wire e_insn_addi = (e_opcode == OpcodeRegImm) && (e_f3 == 3'b000);
  wire e_insn_slti = (e_opcode == OpcodeRegImm) && (e_f3 == 3'b010);
  wire e_insn_sltiu = (e_opcode == OpcodeRegImm) && (e_f3 == 3'b011);
  wire e_insn_xori = (e_opcode == OpcodeRegImm) && (e_f3 == 3'b100);
  wire e_insn_ori = (e_opcode == OpcodeRegImm) && (e_f3 == 3'b110);
  wire e_insn_andi = (e_opcode == OpcodeRegImm) && (e_f3 == 3'b111);
  wire e_insn_slli = (e_opcode == OpcodeRegImm) && (e_f3 == 3'b001) && (e_f7 == 7'd0);
  wire e_insn_srli = (e_opcode == OpcodeRegImm) && (e_f3 == 3'b101) && (e_f7 == 7'd0);
  wire e_insn_srai = (e_opcode == OpcodeRegImm) && (e_f3 == 3'b101) && (e_f7 == 7'b0100000);

  wire e_insn_add = (e_opcode == OpcodeRegReg) && (e_f3 == 3'b000) && (e_f7 == 7'd0);
  wire e_insn_sub = (e_opcode == OpcodeRegReg) && (e_f3 == 3'b000) && (e_f7 == 7'b0100000);
  wire e_insn_sll = (e_opcode == OpcodeRegReg) && (e_f3 == 3'b001) && (e_f7 == 7'd0);
  wire e_insn_slt = (e_opcode == OpcodeRegReg) && (e_f3 == 3'b010) && (e_f7 == 7'd0);
  wire e_insn_sltu = (e_opcode == OpcodeRegReg) && (e_f3 == 3'b011) && (e_f7 == 7'd0);
  wire e_insn_xor = (e_opcode == OpcodeRegReg) && (e_f3 == 3'b100) && (e_f7 == 7'd0);
  wire e_insn_srl = (e_opcode == OpcodeRegReg) && (e_f3 == 3'b101) && (e_f7 == 7'd0);
  wire e_insn_sra = (e_opcode == OpcodeRegReg) && (e_f3 == 3'b101) && (e_f7 == 7'b0100000);
  wire e_insn_or = (e_opcode == OpcodeRegReg) && (e_f3 == 3'b110) && (e_f7 == 7'd0);
  wire e_insn_and = (e_opcode == OpcodeRegReg) && (e_f3 == 3'b111) && (e_f7 == 7'd0);

  wire e_insn_mul = (e_opcode == OpcodeRegReg) && (e_f7 == 7'd1) && (e_f3 == 3'b000);
  wire e_insn_mulh = (e_opcode == OpcodeRegReg) && (e_f7 == 7'd1) && (e_f3 == 3'b001);
  wire e_insn_mulhsu = (e_opcode == OpcodeRegReg) && (e_f7 == 7'd1) && (e_f3 == 3'b010);
  wire e_insn_mulhu = (e_opcode == OpcodeRegReg) && (e_f7 == 7'd1) && (e_f3 == 3'b011);
  wire e_insn_div = (e_opcode == OpcodeRegReg) && (e_f7 == 7'd1) && (e_f3 == 3'b100);
  wire e_insn_divu = (e_opcode == OpcodeRegReg) && (e_f7 == 7'd1) && (e_f3 == 3'b101);
  wire e_insn_rem = (e_opcode == OpcodeRegReg) && (e_f7 == 7'd1) && (e_f3 == 3'b110);
  wire e_insn_remu = (e_opcode == OpcodeRegReg) && (e_f7 == 7'd1) && (e_f3 == 3'b111);

  logic [`REG_SIZE] alu_a, alu_b, alu_result;
  logic alu_cin;
  CarryLookaheadAdder alu_adder (.a(alu_a), .b(alu_b), .cin(alu_cin), .sum(alu_result));

  logic [63:0] mul_ss, mul_su, mul_uu;
  assign mul_ss = $signed({{32{e_rs1_val[31]}}, e_rs1_val}) * $signed({{32{e_rs2_val[31]}}, e_rs2_val});
  assign mul_su = $signed({{32{e_rs1_val[31]}}, e_rs1_val}) * $unsigned({32'b0, e_rs2_val});
  assign mul_uu = $unsigned({32'b0, e_rs1_val}) * $unsigned({32'b0, e_rs2_val});

  logic [`REG_SIZE] e_rd_data;
  logic [`REG_SIZE] e_mem_addr;
  logic e_is_load, e_is_store;
  logic [2:0] e_funct3_mem;
  logic e_div_slow;
  logic e_div_signed;
  logic e_div_is_rem;
  logic e_rs1_neg, e_rs2_neg;

  wire e_div_or_rem = e_insn_div || e_insn_divu || e_insn_rem || e_insn_remu;
  wire e_rs2_zero = (e_rs2_val == 32'd0);

  always_comb begin
    alu_a = 32'd0;
    alu_b = 32'd0;
    alu_cin = 1'b0;
    e_rd_data = 32'd0;
    e_mem_addr = 32'd0;
    e_is_load = 1'b0;
    e_is_store = 1'b0;
    e_funct3_mem = 3'b000;
    e_div_slow = 1'b0;
    e_div_signed = 1'b0;
    e_div_is_rem = 1'b0;
    e_rs1_neg = e_rs1_val[31];
    e_rs2_neg = e_rs2_val[31];

    if (e_insn_lui)
      e_rd_data = execute_state.imm;
    else if (e_insn_auipc) begin
      alu_a = execute_state.pc;
      alu_b = execute_state.imm;
      e_rd_data = alu_result;
    end else if (e_insn_jal)
      e_rd_data = execute_state.pc + 32'd4;
    else if (e_insn_jalr)
      e_rd_data = execute_state.pc + 32'd4;
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
    else if (e_insn_mul)
      e_rd_data = mul_uu[31:0];
    else if (e_insn_mulh)
      e_rd_data = mul_ss[63:32];
    else if (e_insn_mulhsu)
      e_rd_data = mul_su[63:32];
    else if (e_insn_mulhu)
      e_rd_data = mul_uu[63:32];
    else if (e_insn_div) begin
      e_div_signed = 1'b1;
      e_div_is_rem = 1'b0;
      if (e_rs2_zero)
        e_rd_data = 32'hFFFFFFFF;
      else begin
        e_div_slow = 1'b1;
        e_rd_data = 32'd0;
      end
    end else if (e_insn_divu) begin
      e_div_signed = 1'b0;
      e_div_is_rem = 1'b0;
      if (e_rs2_zero)
        e_rd_data = 32'hFFFFFFFF;
      else begin
        e_div_slow = 1'b1;
        e_rd_data = 32'd0;
      end
    end else if (e_insn_rem) begin
      e_div_signed = 1'b1;
      e_div_is_rem = 1'b1;
      if (e_rs2_zero)
        e_rd_data = e_rs1_val;
      else begin
        e_div_slow = 1'b1;
        e_rd_data = 32'd0;
      end
    end else if (e_insn_remu) begin
      e_div_signed = 1'b0;
      e_div_is_rem = 1'b1;
      if (e_rs2_zero)
        e_rd_data = e_rs1_val;
      else begin
        e_div_slow = 1'b1;
        e_rd_data = 32'd0;
      end
    end else if (e_insn_lb || e_insn_lh || e_insn_lw || e_insn_lbu || e_insn_lhu) begin
      e_is_load = 1'b1;
      e_funct3_mem = e_f3;
      alu_a = e_rs1_val;
      alu_b = execute_state.imm;
      e_mem_addr = alu_result;
      e_rd_data = 32'd0;
    end else if (e_insn_sb || e_insn_sh || e_insn_sw) begin
      e_is_store = 1'b1;
      e_funct3_mem = e_f3;
      alu_a = e_rs1_val;
      alu_b = execute_state.imm;
      e_mem_addr = alu_result;
      e_rd_data = 32'd0;
    end
  end

  always_comb begin
    d_rs1_data_bypassed = d_rs1_data;
    d_rs2_data_bypassed = d_rs2_data;
    if (writeback_state.we && writeback_state.rd != 5'd0) begin
      if (writeback_state.rd == insn_rs1) d_rs1_data_bypassed = writeback_state.rd_data;
      if (writeback_state.rd == insn_rs2) d_rs2_data_bypassed = writeback_state.rd_data;
    end
    if (memory_state.we && memory_state.rd != 5'd0) begin
      if (memory_state.rd == insn_rs1) begin
        if (memory_state.is_load)
          d_rs1_data_bypassed = load_data_from_dmem;
        else
          d_rs1_data_bypassed = memory_state.rd_data;
      end
      if (memory_state.rd == insn_rs2) begin
        if (memory_state.is_load)
          d_rs2_data_bypassed = load_data_from_dmem;
        else
          d_rs2_data_bypassed = memory_state.rd_data;
      end
    end
    if (execute_state.we && execute_state.rd != 5'd0 && !exe_is_load) begin
      if (execute_state.rd == insn_rs1) d_rs1_data_bypassed = e_rd_data;
      if (execute_state.rd == insn_rs2) d_rs2_data_bypassed = e_rd_data;
    end
  end

  wire d_is_div_family = (d_opcode == OpcodeRegReg) && (insn_funct7 == 7'd1) &&
      (insn_funct3 inside {3'b100, 3'b101, 3'b110, 3'b111});
  wire d_div_fast = d_is_div_family && (d_rs2_data_bypassed == 32'd0);
  // One divide in flight through the 8-stage divider; a new issue must not reset hold regs.
  assign stall_div_overlap = (div_hold_cnt != 3'd0) && d_is_div_family && !d_div_fast;

  wire [12:0] e_imm_b;
  assign {e_imm_b[12], e_imm_b[10:5]} = e_insn[31:25];
  assign {e_imm_b[4:1], e_imm_b[11]} = e_insn[11:7];
  assign e_imm_b[0] = 1'b0;

  wire [`REG_SIZE] e_branch_target;
  CarryLookaheadAdder e_branch_adder (
    .a(execute_state.pc),
    .b({{19{e_imm_b[12]}}, e_imm_b[12:0]}),
    .cin(1'b0),
    .sum(e_branch_target));

  wire [`REG_SIZE] e_jal_target;
  CarryLookaheadAdder e_jal_adder (
    .a(execute_state.pc),
    .b(execute_state.imm),
    .cin(1'b0),
    .sum(e_jal_target));

  wire [`REG_SIZE] e_jalr_target = (e_rs1_val + execute_state.imm) & ~32'd1;

  always_comb begin
    branch_taken = 1'b0;
    branch_target_actual = e_branch_target;
    if (e_insn_jal) begin
      branch_taken = 1'b1;
      branch_target_actual = e_jal_target;
    end else if (e_insn_jalr) begin
      branch_taken = 1'b1;
      branch_target_actual = e_jalr_target;
    end else if (e_insn_beq)
      branch_taken = (e_rs1_val == e_rs2_val);
    else if (e_insn_bne)
      branch_taken = (e_rs1_val != e_rs2_val);
    else if (e_insn_blt)
      branch_taken = ($signed(e_rs1_val) < $signed(e_rs2_val));
    else if (e_insn_bge)
      branch_taken = ($signed(e_rs1_val) >= $signed(e_rs2_val));
    else if (e_insn_bltu)
      branch_taken = ($unsigned(e_rs1_val) < $unsigned(e_rs2_val));
    else if (e_insn_bgeu)
      branch_taken = ($unsigned(e_rs1_val) >= $unsigned(e_rs2_val));
  end

  logic d_we;
  logic [`REG_SIZE] d_imm_val;
  always_comb begin
    d_we = 1'b0;
    d_imm_val = imm_i_sext;
    if (insn_lui) begin
      d_we = 1'b1;
      d_imm_val = {imm_u, 12'b0};
    end else if (insn_auipc) begin
      d_we = 1'b1;
      d_imm_val = {imm_u, 12'b0};
    end else if (insn_jal) begin
      d_we = 1'b1;
      d_imm_val = imm_j_sext;
    end else if (insn_jalr) begin
      d_we = 1'b1;
      d_imm_val = imm_i_sext;
    end else if (insn_lb || insn_lh || insn_lw || insn_lbu || insn_lhu) begin
      d_we = 1'b1;
      d_imm_val = imm_i_sext;
    end else if (insn_addi | insn_slti | insn_sltiu | insn_xori | insn_ori | insn_andi | insn_slli |
        insn_srli | insn_srai | insn_add | insn_sub | insn_sll | insn_slt | insn_sltu | insn_xor |
        insn_srl | insn_sra | insn_or | insn_and | insn_mul | insn_mulh | insn_mulhsu | insn_mulhu |
        insn_div | insn_divu | insn_rem | insn_remu)
      d_we = 1'b1;
  end

  // Divider is issued when the div insn is in Decode (same cycle it will enter Execute).
  // Operands must come from decode bypasses, not execute_state (still the previous insn).
  logic [`REG_SIZE] d_div_abs_rs1, d_div_abs_rs2;
  logic div_issue_valid;
  always_comb begin
    d_div_abs_rs1 = d_rs1_data_bypassed;
    d_div_abs_rs2 = d_rs2_data_bypassed;
    if (insn_div || insn_rem) begin
      if (d_rs1_data_bypassed[31])
        d_div_abs_rs1 = ~d_rs1_data_bypassed + 32'd1;
      if (d_rs2_data_bypassed[31])
        d_div_abs_rs2 = ~d_rs2_data_bypassed + 32'd1;
    end
  end

  // Hold operands for 7 cycles after issue: first cycle uses combinational issue; divider
  // recomputes from i_dividend every cycle, so inputs must stay valid for all 8 stages.
  logic [`REG_SIZE] div_dend_hold, div_dor_hold;
  always_ff @(posedge clk) begin
    if (rst || branch_taken) begin
      div_hold_cnt <= 3'd0;
    end else if (div_issue_valid) begin
      div_dend_hold <= d_div_abs_rs1;
      div_dor_hold <= d_div_abs_rs2;
      div_hold_cnt <= 3'd7;
    end else if (div_hold_cnt != 3'd0) begin
      div_hold_cnt <= div_hold_cnt - 3'd1;
    end
  end

  wire [`REG_SIZE] div_dend_in = div_issue_valid ? d_div_abs_rs1 :
      (div_hold_cnt != 3'd0 ? div_dend_hold : 32'd0);
  wire [`REG_SIZE] div_dor_in = div_issue_valid ? d_div_abs_rs2 :
      (div_hold_cnt != 3'd0 ? div_dor_hold : 32'd1);

  wire [`REG_SIZE] div_quotient, div_remainder;
  DividerUnsignedPipelined divider (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),
    .flush(branch_taken),
    .i_dividend(div_dend_in),
    .i_divisor(div_dor_in),
    .o_remainder(div_remainder),
    .o_quotient(div_quotient));

  wire insn_xfer_de = !stall_front && !flush_decode;
  assign div_issue_valid =
      insn_xfer_de && (insn_div || insn_divu || insn_rem || insn_remu) && !d_div_fast;

  always_ff @(posedge clk) begin
    if (rst || branch_taken) begin
      div_inflight_valid <= 8'd0;
    end else begin
      div_inflight_valid <= {div_inflight_valid[6:0], div_issue_valid};
    end
  end

  always_ff @(posedge clk) begin
    if (rst || branch_taken) begin
      for (int di = 0; di < 8; di++) div_inflight_rd[di] <= 5'd0;
    end else begin
      div_inflight_rd[7] <= div_inflight_rd[6];
      div_inflight_rd[6] <= div_inflight_rd[5];
      div_inflight_rd[5] <= div_inflight_rd[4];
      div_inflight_rd[4] <= div_inflight_rd[3];
      div_inflight_rd[3] <= div_inflight_rd[2];
      div_inflight_rd[2] <= div_inflight_rd[1];
      div_inflight_rd[1] <= div_inflight_rd[0];
      div_inflight_rd[0] <= div_issue_valid ? insn_rd : 5'd0;
    end
  end

  assign stall_mem_div = memory_state.div_slow && !div_inflight_valid[7];

  // Pipelined divider output is combinational from i_dividend; capture while [7] so a later
  // div_issue_valid cannot corrupt the value used for div_final_rd_data.
  logic [`REG_SIZE] div_q_stable, div_r_stable;
  always_ff @(posedge clk) begin
    if (rst || branch_taken) begin
      div_q_stable <= 32'd0;
      div_r_stable <= 32'd0;
    end else if (div_inflight_valid[7]) begin
      div_q_stable <= div_quotient;
      div_r_stable <= div_remainder;
    end
  end

  stage_memory_t memory_next;

  always_comb begin
    if (flush_decode) begin
      execute_next = '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_TAKEN_BRANCH,
        rs1_data: 0, rs2_data: 0, imm: 0, rd: 0, rs1: 0, rs2: 0, we: 0, rd_data: 0
      };
    end else if (stall_mem_div) begin
      execute_next = execute_state;
    end else if (stall_decode_bubble_execute) begin
      execute_next = '{
        pc: 0,
        insn: 0,
        cycle_status: stall_status,
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

  wire [255:0] x_disasm;
  Disasm #(.PREFIX("X")) disasm_2exe (.insn(execute_state.insn), .disasm(x_disasm));

  /******************/
  /* MEMORY STAGE   */
  /******************/

  wire div_mem_result_valid = memory_state.div_slow && div_inflight_valid[7];
  wire [`REG_SIZE] div_q_use = div_mem_result_valid ? div_quotient : div_q_stable;
  wire [`REG_SIZE] div_r_use = div_mem_result_valid ? div_remainder : div_r_stable;

  logic [`REG_SIZE] div_final_rd_data;
  always_comb begin
    div_final_rd_data = 32'd0;
    if (memory_state.div_slow) begin
      if (memory_state.div_is_rem) begin
        if (memory_state.div_signed) begin
          if (memory_state.rs1_neg)
            div_final_rd_data = ~div_r_use + 32'd1;
          else
            div_final_rd_data = div_r_use;
        end else
          div_final_rd_data = div_r_use;
      end else begin
        if (memory_state.div_signed) begin
          if (memory_state.rs1_neg ^ memory_state.rs2_neg)
            div_final_rd_data = ~div_q_use + 32'd1;
          else
            div_final_rd_data = div_q_use;
        end else
          div_final_rd_data = div_q_use;
      end
    end
  end

  always_comb begin
    memory_next = memory_state;
    if (!stall_mem_div) begin
      memory_next = '{
        pc: execute_state.pc,
        insn: execute_state.insn,
        cycle_status: execute_state.cycle_status,
        alu_result: e_mem_addr,
        rd: execute_state.rd,
        we: execute_state.we,
        rd_data: e_rd_data,
        is_load: e_is_load,
        is_store: e_is_store,
        funct3: e_funct3_mem,
        store_val: e_rs2_val,
        div_slow: e_div_slow && !e_rs2_zero,
        div_signed: e_div_signed,
        div_is_rem: e_div_is_rem,
        rs1_neg: e_rs1_neg,
        rs2_neg: e_rs2_neg,
        rs1_orig: e_rs1_val,
        rs2_shadow: execute_state.rs2
      };
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      memory_state <= '{
        pc: 0, insn: 0, cycle_status: CYCLE_RESET,
        alu_result: 0, rd: 0, we: 0, rd_data: 0,
        is_load: 0, is_store: 0, funct3: 0, store_val: 0,
        div_slow: 0, div_signed: 0, div_is_rem: 0, rs1_neg: 0, rs2_neg: 0, rs1_orig: 0,
        rs2_shadow: 0
      };
    end else if (stall_mem_div) begin
      memory_state <= memory_state;
    end else begin
      memory_state <= memory_next;
    end
  end

  logic [`REG_SIZE] wm_store_data;
  always_comb begin
    wm_store_data = memory_state.store_val;
    if (writeback_state.we && writeback_state.rd != 5'd0 &&
        writeback_state.rd == memory_state.rs2_shadow)
      wm_store_data = writeback_state.rd_data;
  end

  always_comb begin
    addr_to_dmem = {memory_state.alu_result[31:2], 2'b00};
    store_data_to_dmem = 32'd0;
    store_we_to_dmem = 4'b0;
    if (memory_state.is_store) begin
      store_data_to_dmem = wm_store_data;
      if (memory_state.funct3 == 3'b010)
        store_we_to_dmem = 4'b1111;
      else if (memory_state.funct3 == 3'b001) begin
        store_data_to_dmem = (memory_state.alu_result[1]) ? {wm_store_data[15:0], 16'b0} :
            {16'b0, wm_store_data[15:0]};
        store_we_to_dmem = (memory_state.alu_result[1]) ? 4'b1100 : 4'b0011;
      end else if (memory_state.funct3 == 3'b000) begin
        store_data_to_dmem = {4{wm_store_data[7:0]}};
        case (memory_state.alu_result[1:0])
          2'b00: store_we_to_dmem = 4'b0001;
          2'b01: store_we_to_dmem = 4'b0010;
          2'b10: store_we_to_dmem = 4'b0100;
          2'b11: store_we_to_dmem = 4'b1000;
        endcase
      end
    end
  end

  /*******************/
  /* WRITEBACK STAGE */
  /*******************/

  stage_writeback_t writeback_next;

  logic [`REG_SIZE] loaded_word;
  always_comb begin
    loaded_word = load_data_from_dmem;
    if (memory_state.is_load) begin
      unique case (memory_state.funct3)
        3'b000:
          unique case (memory_state.alu_result[1:0])
            2'b00: loaded_word = {{24{load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
            2'b01: loaded_word = {{24{load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
            2'b10: loaded_word = {{24{load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
            2'b11: loaded_word = {{24{load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
          endcase
        3'b001:
          loaded_word = (memory_state.alu_result[1]) ? {{16{load_data_from_dmem[31]}},
              load_data_from_dmem[31:16]} : {{16{load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};
        3'b010: loaded_word = load_data_from_dmem;
        3'b100:
          unique case (memory_state.alu_result[1:0])
            2'b00: loaded_word = {24'b0, load_data_from_dmem[7:0]};
            2'b01: loaded_word = {24'b0, load_data_from_dmem[15:8]};
            2'b10: loaded_word = {24'b0, load_data_from_dmem[23:16]};
            2'b11: loaded_word = {24'b0, load_data_from_dmem[31:24]};
          endcase
        3'b101:
          loaded_word = (memory_state.alu_result[1]) ? {16'b0, load_data_from_dmem[31:16]} :
              {16'b0, load_data_from_dmem[15:0]};
        default: loaded_word = load_data_from_dmem;
      endcase
    end
  end

  always_comb begin
    writeback_next = '{
      pc: memory_state.pc,
      insn: memory_state.insn,
      cycle_status: memory_state.cycle_status,
      rd: memory_state.rd,
      we: memory_state.we,
      rd_data: memory_state.rd_data
    };
    if (memory_state.is_load)
      writeback_next.rd_data = loaded_word;
    if (memory_state.div_slow && !div_inflight_valid[7]) begin
      writeback_next.pc = 32'd0;
      writeback_next.insn = 32'd0;
      writeback_next.cycle_status = CYCLE_DIV;
      writeback_next.we = 1'b0;
      writeback_next.rd = 5'd0;
      writeback_next.rd_data = 32'd0;
    end else if (memory_state.div_slow && div_inflight_valid[7]) begin
      writeback_next.rd_data = div_final_rd_data;
      writeback_next.cycle_status = memory_state.cycle_status;
    end
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
    input wire rst,
    input wire clk,
    input wire [`REG_SIZE] pc_to_imem,
    output logic [`REG_SIZE] insn_from_imem,
    input wire [`REG_SIZE] addr_to_dmem,
    output logic [`REG_SIZE] load_data_from_dmem,
    input wire [`REG_SIZE] store_data_to_dmem,
    input wire [3:0] store_we_to_dmem
);

  logic [`REG_SIZE] mem_array[NUM_WORDS];

`ifdef SYNTHESIS
  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end
`endif

  always_comb begin
    assert (pc_to_imem[1:0] == 2'b00);
    assert (addr_to_dmem[1:0] == 2'b00);
  end

  localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam int AddrLsb = 2;

  assign insn_from_imem = mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];

  // Combinational read so pipeline stages (load in M, dependent insn in E) see data for the
  // same addr_to_dmem in the same half-cycle; negedge register was one cycle stale for EX bypass.
  assign load_data_from_dmem = mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];

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
    end
  end
endmodule

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

  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
      .rst                (rst),
      .clk                (clk),
      .pc_to_imem         (pc_to_imem),
      .insn_from_imem     (insn_from_imem),
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
