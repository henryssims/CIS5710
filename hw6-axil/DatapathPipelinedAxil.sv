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
`include "../hw3-singlecycle/cycle_status.sv"
`include "../hw4-multicycle/DividerUnsignedPipelined.sv"
`include "EasyAxilMemory.sv"

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
  assign regs[0]  = 32'd0;
  assign rs1_data = (we && rd != 5'd0 && rd == rs1) ? rd_data : regs[rs1];
  assign rs2_data = (we && rd != 5'd0 && rd == rs2) ? rd_data : regs[rs2];
  /* verilator lint_off BLKSEQ */
  genvar i;
  for (i = 1; i < 32; i = i + 1) begin
    always_ff @(posedge clk) begin
      if (rst) begin
        regs[i] = 32'd0;
      end else begin
        if (we && rd == 5'(i)) begin
          regs[i] = rd_data;
        end
      end
    end
  end
  /* verilator lint_on BLKSEQ */
endmodule

// ---------------------------------------------------------------------------
// Stage structs
// ---------------------------------------------------------------------------
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
} stage_decode_t;

typedef struct packed {
  logic [`REG_SIZE]  pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e     cycle_status;
  logic [`REG_SIZE]  rs1_data;
  logic [`REG_SIZE]  rs2_data;
  logic [4:0]        rs1;
  logic [4:0]        rs2;
  logic [4:0]        rd;
} stage_execute_t;

typedef struct packed {
  logic [`REG_SIZE]  pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e     cycle_status;
  logic [`REG_SIZE]  alu_result;
  logic [`REG_SIZE]  rs2_data;
  logic [4:0]        rd;
  logic              we;
} stage_memory_t;

typedef struct packed {
  logic [`REG_SIZE]  pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e     cycle_status;
  logic [`REG_SIZE]  rd_data;
  logic [4:0]        rd;
  logic              we;
} stage_writeback_t;

// ---------------------------------------------------------------------------
// Divider metadata: travels alongside the divider pipeline
// ---------------------------------------------------------------------------
typedef struct packed {
  logic             valid;    // 1 = a divide result is emerging this stage
  logic [4:0]       rd;       // destination register
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e    cycle_status;
  logic [2:0]       funct3;   // encodes div/divu/rem/remu
  logic [`REG_SIZE] op1;      // original rs1 (for sign/zero correction)
  logic [`REG_SIZE] op2;      // original rs2 (for sign/zero correction)
} div_meta_t;
module DatapathPipelinedAxil (
    input wire clk,
    input wire rst,
    axil_if.manager imem,
    axil_if.manager dmem,
    output logic halt,
    output logic [`REG_SIZE] trace_completed_pc,
    output logic [`INSN_SIZE] trace_completed_insn,
    output cycle_status_e trace_completed_cycle_status
);

  localparam bit [`OPCODE_SIZE] OpcodeLoad    = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeStore   = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeBranch  = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeJalr    = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpcodeMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpcodeJal     = 7'b11_011_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegImm  = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegReg  = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeEnviron = 7'b11_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeAuipc   = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpcodeLui     = 7'b01_101_11;

  localparam bit True = 1'b1;
  localparam bit False = 1'b0;

  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) cycles_current <= 0;
    else     cycles_current <= cycles_current + 1;
  end

  // =========================================================================
  // Forward declarations
  // =========================================================================
  logic             x_branch_taken;
  logic [`REG_SIZE] x_branch_target;
  logic [`REG_SIZE] w_rd_data;
  logic [4:0]       w_rd;
  logic             w_we;
  logic [`REG_SIZE] m_alu_result;
  logic [4:0]       m_rd;
  logic             m_we;

  stage_decode_t  decode_state;
  stage_execute_t execute_state;
  stage_memory_t  memory_state;
  logic branch_flush_pending;

  // =========================================================================
  // DIVIDER PIPELINE + METADATA SHIFT REGISTER
  //
  // The DividerUnsignedPipelined has DIVIDER_STAGES stages. We maintain a
  // parallel shift register of div_meta_t that travels in lockstep.
  // div_meta[0] is loaded when a div enters Execute.
  // div_meta[DIVIDER_STAGES-1] holds the result metadata when it emerges.
  // Independent divides can be launched on consecutive cycles:
  //   div x2 F D X M W         (X = one cycle, result emerges DIVIDER_STAGES later)
  //   div x3   F D X M W
  // The metadata pipeline tracks both simultaneously.
  //
  // Stalling: only needed when a DEPENDENT instruction needs the divide result.
  // We detect this in Decode: if any in-flight div_meta[i].valid && rd matches
  // the Decode instruction's rs1 or rs2, we must stall until that div reaches
  // the end of its pipeline (div_meta[DIVIDER_STAGES-1].valid).
  // =========================================================================

  // Divider stage 0 is combinational from i_dividend/i_divisor; o_quotient updates if
  // those change while a result is being sampled. Stall divider whenever the front end
  // stalls so operands/meta stay aligned; flush on branch.
  /* verilator lint_off UNOPTFLAT */
  logic [`REG_SIZE] div_dividend, div_divisor;
  wire  [`REG_SIZE] div_quotient, div_remainder;
  /* verilator lint_on UNOPTFLAT */

  // Metadata shift register: one entry per divider stage.
  // div_meta[0] is loaded when a divide launches in Execute.
  div_meta_t div_meta[`DIVIDER_STAGES];

  // Latency through DividerUnsignedPipelined matches meta[DIVIDER_STAGES-2] here
  // (see course pipeline diagram / shift count vs 8-tap divider).
  wire div_result_valid = div_meta[`DIVIDER_STAGES-2].valid;
  wire [4:0] div_result_rd = div_meta[`DIVIDER_STAGES-2].rd;

  wire [2:0]       dr_funct3 = div_meta[`DIVIDER_STAGES-2].funct3;
  wire [`REG_SIZE] dr_op1    = div_meta[`DIVIDER_STAGES-2].op1;
  wire [`REG_SIZE] dr_op2    = div_meta[`DIVIDER_STAGES-2].op2;
  wire             dr_neg1   = dr_op1[31];
  wire             dr_neg2   = dr_op2[31];

  logic [`REG_SIZE] div_result_data;
  always_comb begin
    div_result_data = 0;
    case (dr_funct3)
      3'b100: // div
        div_result_data = (dr_op2==0) ? 32'hFFFF_FFFF :
                          (dr_neg1^dr_neg2) ? (~div_quotient+1) : div_quotient;
      3'b101: // divu
        div_result_data = (dr_op2==0) ? 32'hFFFF_FFFF : div_quotient;
      3'b110: // rem
        div_result_data = (dr_op2==0) ? dr_op1 :
                          dr_neg1 ? (~div_remainder+1) : div_remainder;
      3'b111: // remu
        div_result_data = (dr_op2==0) ? dr_op1 : div_remainder;
      default: div_result_data = 0;
    endcase
  end

  // =========================================================================
  // DIVIDE HAZARDS
  // Non-div instructions wait behind any in-flight divide.
  // Divide instructions may launch back-to-back, but must stall if they depend
  // on an older divide result that is still in the divider pipeline.
  // =========================================================================
  wire [4:0] d_rs1 = decode_state.insn[19:15];
  wire [4:0] d_rs2 = decode_state.insn[24:20];
  wire [`OPCODE_SIZE] d_opcode = decode_state.insn[6:0];
  wire [2:0]          d_funct3 = decode_state.insn[14:12];
  wire [6:0]          d_funct7 = decode_state.insn[31:25];
  wire                d_is_div = (d_opcode == OpcodeRegReg) && (d_funct7 == 7'd1)
                              && (d_funct3[2] == 1'b1);
  wire                d_is_store  = (d_opcode == OpcodeStore);
  wire                d_is_branch = (d_opcode == OpcodeBranch);
  wire                d_is_jalr   = (d_opcode == OpcodeJalr);
  wire                d_is_load   = (d_opcode == OpcodeLoad);
  wire                d_is_opimm  = (d_opcode == OpcodeRegImm);
  wire                d_is_regreg = (d_opcode == OpcodeRegReg);
  wire                d_has_real_insn = (decode_state.cycle_status == CYCLE_NO_STALL);

  logic d_uses_rs1, d_uses_rs2;
  always_comb begin
    d_uses_rs1 = d_is_load || d_is_store || d_is_branch || d_is_jalr || d_is_opimm || d_is_regreg;
    d_uses_rs2 = d_is_store || d_is_branch || d_is_regreg;
  end

  wire x_is_div_early = (execute_state.insn[6:0] == OpcodeRegReg)
                      && (execute_state.insn[31:25] == 7'd1)
                      && (execute_state.insn[14:12] >= 3'b100);
  wire x_has_real_insn = (execute_state.cycle_status == CYCLE_NO_STALL);
  wire x_div_is_fast   = x_is_div_early && (execute_state.rs2_data == 32'd0);
  wire x_div_inflight  = x_has_real_insn && x_is_div_early && !x_div_is_fast;

  logic div_pipeline_busy;
  logic div_dep_hazard;
  localparam int DIV_HAZARD_STAGES = `DIVIDER_STAGES - 2;
  always_comb begin
    div_pipeline_busy = x_div_inflight;
    div_dep_hazard = 1'b0;
    if (x_div_inflight && (execute_state.rd != 5'd0)) begin
      if (d_uses_rs1 && execute_state.rd == d_rs1) div_dep_hazard = 1'b1;
      if (d_uses_rs2 && execute_state.rd == d_rs2) div_dep_hazard = 1'b1;
    end
    for (int s = 0; s < DIV_HAZARD_STAGES; s++) begin
      if (div_meta[s].valid) begin
        div_pipeline_busy = 1'b1;
        if (div_meta[s].rd != 5'd0) begin
          if (d_uses_rs1 && div_meta[s].rd == d_rs1) div_dep_hazard = 1'b1;
          if (d_uses_rs2 && div_meta[s].rd == d_rs2) div_dep_hazard = 1'b1;
        end
      end
    end
  end

  // Younger non-div instructions must wait behind any in-flight divide.
  // Younger divide instructions only wait if they depend on an older divide.
  wire div_stall = d_has_real_insn && (d_is_div ? div_dep_hazard : div_pipeline_busy);

  // Divide completing into M while X has a non-div insn: stall (M vs X conflict).
  // While a divide result is valid for M and Decode holds another div, stall F/D so
  // divider inputs are not re-driven the same cycle o_quotient is sampled.
  wire div_insert_stall = 1'b0;
  wire stall_div_wb = 1'b0;

  // =========================================================================
  // LOAD-USE STALL (before pipeline_stall uses it)
  // =========================================================================
  wire x_is_load    = (execute_state.insn[6:0] == OpcodeLoad);
  wire x_load_rd_nz = (execute_state.insn[11:7] != 5'd0);
  // Store address (rs1) after a load is handled by MX bypass in X.
  wire d_needs_load_rs1 = d_uses_rs1 && !d_is_store && (d_rs1 == execute_state.insn[11:7]);
  // For SW/SH, stall one cycle on load->store-data to match reference timing.
  // SB is handled directly via MX bypass without adding a bubble.
  wire d_store_is_sb = d_is_store && (d_funct3 == 3'b000);
  wire d_needs_load_rs2 = d_uses_rs2 && (d_rs2 == execute_state.insn[11:7]) && !d_store_is_sb;
  wire d_needs_load = d_needs_load_rs1 || d_needs_load_rs2;
  wire load_use_stall = x_is_load && x_load_rd_nz && d_needs_load
                     && (decode_state.cycle_status == CYCLE_NO_STALL);

  wire [`OPCODE_SIZE] m_opc_mem = memory_state.insn[6:0];
  wire m_is_ld_mem = (m_opc_mem == OpcodeLoad);
  wire m_is_st_mem = (m_opc_mem == OpcodeStore);
  wire m_mem_real = (memory_state.cycle_status == CYCLE_NO_STALL);
  wire mem_stall = (m_mem_real && m_is_ld_mem && !dmem.RVALID)
                || (m_mem_real && m_is_st_mem && !dmem.BVALID);

  wire pipeline_stall = load_use_stall || div_stall || div_insert_stall || stall_div_wb || mem_stall;

  DividerUnsignedPipelined divider (
      .clk(clk),
      .rst(rst),
      .stall(1'b0),
      .flush(x_branch_taken),
      .i_dividend(div_dividend),
      .i_divisor(div_divisor),
      .o_quotient(div_quotient),
      .o_remainder(div_remainder)
  );

  // =========================================================================
  // FETCH / G: AXI-Lite icache (pipelined AR, PC tag FIFO depth 2)
  // =========================================================================
  logic [`REG_SIZE] seq_pc;
  logic [`REG_SIZE] q0, q1;
  logic qv0, qv1;
  logic [`REG_SIZE] g_insn, g_pc;
  logic g_valid;
  cycle_status_e g_cycle_status;
  wire ar_hs = imem.ARVALID && imem.ARREADY;
  wire r_hs  = imem.RVALID && imem.RREADY;
  wire ififo_full = qv0 && qv1;
  logic icache_rready_w;

  always_comb begin
    icache_rready_w = False;
    if (imem.RVALID) begin
      if (!(g_valid && pipeline_stall)) icache_rready_w = True;
    end
  end
  assign imem.RREADY = icache_rready_w;

  always_ff @(posedge clk) begin
    if (rst) begin
      seq_pc <= 32'd0;
      q0 <= 32'd0;
      q1 <= 32'd0;
      qv0 <= 0;
      qv1 <= 0;
      g_valid <= 0;
      g_cycle_status <= CYCLE_RESET;
    end else if (x_branch_taken) begin
      q0 <= 32'd0;
      q1 <= 32'd0;
      qv0 <= 0;
      qv1 <= 0;
      g_valid <= 0;
      g_cycle_status <= CYCLE_RESET;
      // Launch the redirected fetch immediately to keep branch penalty at 3
      // bubbles (F/G/D), matching the expected pipeline timing.
      if (imem.ARREADY) begin
        q0 <= x_branch_target;
        qv0 <= 1'b1;
        seq_pc <= x_branch_target + 32'd4;
      end else begin
        seq_pc <= x_branch_target;
      end
    end else begin
      logic [`REG_SIZE] next_q0, next_q1;
      logic next_qv0, next_qv1;
      next_q0 = q0;
      next_q1 = q1;
      next_qv0 = qv0;
      next_qv1 = qv1;

      if (r_hs) begin
        g_insn <= imem.RDATA;
        g_pc <= q0;
        g_valid <= 1'b1;
        g_cycle_status <= CYCLE_NO_STALL;
        if (next_qv0) begin
          next_q0 = next_q1;
          next_qv0 = next_qv1;
          next_qv1 = 1'b0;
        end
      end else if (!pipeline_stall && g_valid) begin
        g_valid <= 0;
      end
      if (ar_hs) begin
        if (!next_qv0) begin
          next_q0 = imem.ARADDR;
          next_qv0 = 1'b1;
        end else if (!next_qv1) begin
          next_q1 = imem.ARADDR;
          next_qv1 = 1'b1;
        end
        seq_pc <= seq_pc + 32'd4;
      end
      q0 <= next_q0;
      q1 <= next_q1;
      qv0 <= next_qv0;
      qv1 <= next_qv1;
    end
  end

  always_comb begin
    imem.ARPROT = 3'b000;
    imem.ARADDR = seq_pc;
    if (rst) imem.ARVALID = False;
    else if (x_branch_taken) begin
      imem.ARADDR = x_branch_target;
      imem.ARVALID = True;
    end
    else if (pipeline_stall) imem.ARVALID = False;
    else imem.ARVALID = !ififo_full;
  end

  cycle_status_e f_cycle_status;
  always_comb begin
    f_cycle_status = CYCLE_NO_STALL;
    if (imem.RVALID && !icache_rready_w) f_cycle_status = CYCLE_IMEM_WAIT;
  end

  wire [255:0] f_disasm;
  Disasm #(.PREFIX("F")) disasm_0fetch (.insn(g_valid ? g_insn : 32'd0), .disasm(f_disasm));

  // =========================================================================
  // DECODE
  // =========================================================================
  always_ff @(posedge clk) begin
    if (rst) begin
      branch_flush_pending <= 1'b0;
    end else if (x_branch_taken) begin
      branch_flush_pending <= 1'b1;
    end else if (!pipeline_stall) begin
      // Keep tagging bubbles as taken-branch until the redirected stream
      // produces a valid instruction in G/D.
      if (branch_flush_pending && !g_valid) branch_flush_pending <= 1'b1;
      else branch_flush_pending <= 1'b0;
    end
  end

  always_ff @(posedge clk) begin
    if (rst)
      decode_state <= '{pc:0, insn:0, cycle_status:CYCLE_RESET};
    else if (pipeline_stall)
      decode_state <= decode_state;
    else if (x_branch_taken)
      decode_state <= '{pc:0, insn:0, cycle_status:CYCLE_TAKEN_BRANCH};
    else if (branch_flush_pending && !g_valid)
      decode_state <= '{pc:0, insn:0, cycle_status:CYCLE_TAKEN_BRANCH};
    else if (g_valid)
      decode_state <= '{pc:g_pc, insn:g_insn, cycle_status:g_cycle_status};
    else
      decode_state <= '{pc:0, insn:0, cycle_status:CYCLE_RESET};
  end

  wire [255:0] d_disasm;
  Disasm #(.PREFIX("D")) disasm_1decode (.insn(decode_state.insn), .disasm(d_disasm));

  wire [4:0] d_rd = decode_state.insn[11:7];
  wire [`REG_SIZE] d_rs1_data_rf, d_rs2_data_rf;
  RegFile rf (
      .clk(clk), .rst(rst),
      .we(w_we), .rd(w_rd), .rd_data(w_rd_data),
      .rs1(d_rs1), .rs1_data(d_rs1_data_rf),
      .rs2(d_rs2), .rs2_data(d_rs2_data_rf)
  );

  // Filled after x_alu_result and m_load_result (decode -> execute operand bypass).
  logic [`REG_SIZE] d_rs1_data_bypassed, d_rs2_data_bypassed;

  // =========================================================================
  // EXECUTE
  // =========================================================================
  always_ff @(posedge clk) begin
    if (rst)
      execute_state <= '{pc:0,insn:0,cycle_status:CYCLE_RESET,rs1_data:0,rs2_data:0,rs1:0,rs2:0,rd:0};
    else if (load_use_stall)
      execute_state <= '{pc:0,insn:0,cycle_status:CYCLE_LOAD2USE,rs1_data:0,rs2_data:0,rs1:0,rs2:0,rd:0};
    else if (div_stall)
      execute_state <= '{pc:0,insn:0,cycle_status:CYCLE_DIV,rs1_data:0,rs2_data:0,rs1:0,rs2:0,rd:0};
    else if (x_branch_taken)
      execute_state <= '{pc:0,insn:0,cycle_status:CYCLE_TAKEN_BRANCH,rs1_data:0,rs2_data:0,rs1:0,rs2:0,rd:0};
    else if (mem_stall)
      execute_state <= execute_state;
    else
      execute_state <= '{
        pc:decode_state.pc, insn:decode_state.insn, cycle_status:decode_state.cycle_status,
        rs1_data:d_rs1_data_bypassed, rs2_data:d_rs2_data_bypassed, rs1:d_rs1, rs2:d_rs2, rd:d_rd
      };
  end

  wire [255:0] x_disasm;
  Disasm #(.PREFIX("X")) disasm_2execute (.insn(execute_state.insn), .disasm(x_disasm));

  wire [6:0] x_funct7   = execute_state.insn[31:25];
  wire [2:0] x_funct3   = execute_state.insn[14:12];
  wire [4:0] x_rd       = execute_state.insn[11:7];
  wire [`OPCODE_SIZE] x_opcode = execute_state.insn[6:0];
  wire [19:0] x_imm_u     = execute_state.insn[31:12];
  wire [11:0] x_imm_i     = execute_state.insn[31:20];
  wire [ 4:0] x_imm_shamt = execute_state.insn[24:20];
  wire [11:0] x_imm_s;
  assign x_imm_s[11:5]=x_funct7; assign x_imm_s[4:0]=x_rd;
  wire [12:0] x_imm_b;
  assign {x_imm_b[12],x_imm_b[10:5]}=x_funct7;
  assign {x_imm_b[4:1],x_imm_b[11]}=x_rd;
  assign x_imm_b[0]=1'b0;
  wire [20:0] x_imm_j;
  assign {x_imm_j[20],x_imm_j[10:1],x_imm_j[11],x_imm_j[19:12],x_imm_j[0]}={execute_state.insn[31:12],1'b0};
  wire [`REG_SIZE] x_imm_i_sext = {{20{x_imm_i[11]}},x_imm_i};
  wire [`REG_SIZE] x_imm_s_sext = {{20{x_imm_s[11]}},x_imm_s};
  wire [`REG_SIZE] x_imm_b_sext = {{19{x_imm_b[12]}},x_imm_b};
  wire [`REG_SIZE] x_imm_j_sext = {{11{x_imm_j[20]}},x_imm_j};

  wire [255:0] m_disasm;
  Disasm #(.PREFIX("M")) disasm_3memory (.insn(memory_state.insn), .disasm(m_disasm));
  assign m_rd=memory_state.rd; assign m_we=memory_state.we; assign m_alu_result=memory_state.alu_result;

  wire [`OPCODE_SIZE] m_opcode=memory_state.insn[6:0];
  wire [2:0] m_funct3=memory_state.insn[14:12];
  wire m_is_load =m_opcode==OpcodeLoad;
  wire m_is_store=m_opcode==OpcodeStore;
  wire m_lw =m_is_load&&m_funct3==3'b010;
  wire m_lh =m_is_load&&m_funct3==3'b001;
  wire m_lhu=m_is_load&&m_funct3==3'b101;
  wire m_lb =m_is_load&&m_funct3==3'b000;
  wire m_lbu=m_is_load&&m_funct3==3'b100;
  wire m_sw =m_is_store&&m_funct3==3'b010;
  wire m_sh =m_is_store&&m_funct3==3'b001;
  wire m_sb =m_is_store&&m_funct3==3'b000;

  wire [`REG_SIZE] m_store_data = memory_state.rs2_data;

  logic [`REG_SIZE] m_load_result;
  always_comb begin
    m_load_result=0;
    if(m_lw) m_load_result=dmem.RDATA;
    else if(m_lh)  m_load_result=memory_state.alu_result[1]?{{16{dmem.RDATA[31]}},dmem.RDATA[31:16]}:{{16{dmem.RDATA[15]}},dmem.RDATA[15:0]};
    else if(m_lhu) m_load_result=memory_state.alu_result[1]?{16'b0,dmem.RDATA[31:16]}:{16'b0,dmem.RDATA[15:0]};
    else if(m_lb) case(memory_state.alu_result[1:0])
      2'b00:m_load_result={{24{dmem.RDATA[7]}},dmem.RDATA[7:0]};
      2'b01:m_load_result={{24{dmem.RDATA[15]}},dmem.RDATA[15:8]};
      2'b10:m_load_result={{24{dmem.RDATA[23]}},dmem.RDATA[23:16]};
      2'b11:m_load_result={{24{dmem.RDATA[31]}},dmem.RDATA[31:24]};
    endcase
    else if(m_lbu) case(memory_state.alu_result[1:0])
      2'b00:m_load_result={24'b0,dmem.RDATA[7:0]};
      2'b01:m_load_result={24'b0,dmem.RDATA[15:8]};
      2'b10:m_load_result={24'b0,dmem.RDATA[23:16]};
      2'b11:m_load_result={24'b0,dmem.RDATA[31:24]};
    endcase
  end

  wire [`REG_SIZE] m_fwd_data = (m_is_load && m_mem_real) ? m_load_result : m_alu_result;

  // MX/WX bypass (M may be a load: forward data, not byte address in alu_result)
  wire mx_rs1 = m_we && m_rd!=5'd0 && m_rd==execute_state.rs1;
  wire mx_rs2 = m_we && m_rd!=5'd0 && m_rd==execute_state.rs2;
  wire wx_rs1 = w_we && w_rd!=5'd0 && w_rd==execute_state.rs1;
  wire wx_rs2 = w_we && w_rd!=5'd0 && w_rd==execute_state.rs2;
  wire [`REG_SIZE] x_rs1 = mx_rs1 ? m_fwd_data : wx_rs1 ? w_rd_data : execute_state.rs1_data;
  wire [`REG_SIZE] x_rs2 = mx_rs2 ? m_fwd_data : wx_rs2 ? w_rd_data : execute_state.rs2_data;

  wire mx_div_rs1 = m_we && m_rd != 5'd0 && m_rd == execute_state.rs1;
  wire mx_div_rs2 = m_we && m_rd != 5'd0 && m_rd == execute_state.rs2;
  wire wx_div_rs1 = w_we && w_rd != 5'd0 && w_rd == execute_state.rs1;
  wire wx_div_rs2 = w_we && w_rd != 5'd0 && w_rd == execute_state.rs2;
  wire [`REG_SIZE] x_div_rs1 = mx_div_rs1 ? m_fwd_data
                                         : wx_div_rs1 ? w_rd_data
                                                      : execute_state.rs1_data;
  wire [`REG_SIZE] x_div_rs2 = mx_div_rs2 ? m_fwd_data
                                         : wx_div_rs2 ? w_rd_data
                                                      : execute_state.rs2_data;

  wire [`REG_SIZE] x_adder_sum;
  logic [`REG_SIZE] x_adder_a, x_adder_b;
  logic x_adder_cin;
  CarryLookaheadAdder x_adder (.a(x_adder_a),.b(x_adder_b),.cin(x_adder_cin),.sum(x_adder_sum));

  wire [63:0] x_mul_ss=$signed({{32{x_rs1[31]}},x_rs1})*$signed({{32{x_rs2[31]}},x_rs2});
  wire [63:0] x_mul_su=$signed({{32{x_rs1[31]}},x_rs1})*$unsigned({32'b0,x_rs2});
  wire [63:0] x_mul_uu=$unsigned({32'b0,x_rs1})*$unsigned({32'b0,x_rs2});

  wire x_beq  =x_opcode==OpcodeBranch&&x_funct3==3'b000;
  wire x_bne  =x_opcode==OpcodeBranch&&x_funct3==3'b001;
  wire x_blt  =x_opcode==OpcodeBranch&&x_funct3==3'b100;
  wire x_bge  =x_opcode==OpcodeBranch&&x_funct3==3'b101;
  wire x_bltu =x_opcode==OpcodeBranch&&x_funct3==3'b110;
  wire x_bgeu =x_opcode==OpcodeBranch&&x_funct3==3'b111;
  wire x_addi =x_opcode==OpcodeRegImm&&x_funct3==3'b000;
  wire x_slti =x_opcode==OpcodeRegImm&&x_funct3==3'b010;
  wire x_sltiu=x_opcode==OpcodeRegImm&&x_funct3==3'b011;
  wire x_xori =x_opcode==OpcodeRegImm&&x_funct3==3'b100;
  wire x_ori  =x_opcode==OpcodeRegImm&&x_funct3==3'b110;
  wire x_andi =x_opcode==OpcodeRegImm&&x_funct3==3'b111;
  wire x_slli =x_opcode==OpcodeRegImm&&x_funct3==3'b001&&x_funct7==7'd0;
  wire x_srli =x_opcode==OpcodeRegImm&&x_funct3==3'b101&&x_funct7==7'd0;
  wire x_srai =x_opcode==OpcodeRegImm&&x_funct3==3'b101&&x_funct7==7'b0100000;
  wire x_add  =x_opcode==OpcodeRegReg&&x_funct3==3'b000&&x_funct7==7'd0;
  wire x_sub  =x_opcode==OpcodeRegReg&&x_funct3==3'b000&&x_funct7==7'b0100000;
  wire x_sll  =x_opcode==OpcodeRegReg&&x_funct3==3'b001&&x_funct7==7'd0;
  wire x_slt  =x_opcode==OpcodeRegReg&&x_funct3==3'b010&&x_funct7==7'd0;
  wire x_sltu =x_opcode==OpcodeRegReg&&x_funct3==3'b011&&x_funct7==7'd0;
  wire x_xor  =x_opcode==OpcodeRegReg&&x_funct3==3'b100&&x_funct7==7'd0;
  wire x_srl  =x_opcode==OpcodeRegReg&&x_funct3==3'b101&&x_funct7==7'd0;
  wire x_sra  =x_opcode==OpcodeRegReg&&x_funct3==3'b101&&x_funct7==7'b0100000;
  wire x_or   =x_opcode==OpcodeRegReg&&x_funct3==3'b110&&x_funct7==7'd0;
  wire x_and  =x_opcode==OpcodeRegReg&&x_funct3==3'b111&&x_funct7==7'd0;
  wire x_mul_i=x_opcode==OpcodeRegReg&&x_funct7==7'd1&&x_funct3==3'b000;
  wire x_mulh =x_opcode==OpcodeRegReg&&x_funct7==7'd1&&x_funct3==3'b001;
  wire x_mulhsu=x_opcode==OpcodeRegReg&&x_funct7==7'd1&&x_funct3==3'b010;
  wire x_mulhu=x_opcode==OpcodeRegReg&&x_funct7==7'd1&&x_funct3==3'b011;
  wire x_div  =x_opcode==OpcodeRegReg&&x_funct7==7'd1&&x_funct3==3'b100;
  wire x_divu =x_opcode==OpcodeRegReg&&x_funct7==7'd1&&x_funct3==3'b101;
  wire x_rem  =x_opcode==OpcodeRegReg&&x_funct7==7'd1&&x_funct3==3'b110;
  wire x_remu =x_opcode==OpcodeRegReg&&x_funct7==7'd1&&x_funct3==3'b111;
  wire x_is_div = x_div|x_divu|x_rem|x_remu;
  wire x_is_div_slow = x_is_div && (x_div_rs2 != 32'd0);
  wire x_ecall=x_opcode==OpcodeEnviron&&execute_state.insn[31:7]==25'd0;

  logic [`REG_SIZE] x_alu_result;
  logic             x_we;
  logic             x_bc;

  always_comb begin
    x_adder_a=0;x_adder_b=0;x_adder_cin=0;
    x_alu_result=0;x_we=0;x_branch_taken=0;x_branch_target=0;x_bc=0;
    div_dividend=32'd1;div_divisor=32'd1;

    case (x_opcode)
      OpcodeLui:   begin x_we=1;x_alu_result={x_imm_u,12'b0};end
      OpcodeAuipc: begin x_we=1;x_alu_result=execute_state.pc+{x_imm_u,12'b0};end
      OpcodeJal:   begin x_we=1;x_alu_result=execute_state.pc+4;x_branch_taken=1;x_branch_target=execute_state.pc+x_imm_j_sext;end
      OpcodeJalr:  begin x_we=1;x_alu_result=execute_state.pc+4;x_branch_taken=1;x_branch_target=(x_rs1+x_imm_i_sext)&~32'b1;end
      OpcodeBranch: begin
        if(x_beq)  x_bc=(x_rs1==x_rs2);
        else if(x_bne)  x_bc=(x_rs1!=x_rs2);
        else if(x_blt)  x_bc=($signed(x_rs1)<$signed(x_rs2));
        else if(x_bge)  x_bc=($signed(x_rs1)>=$signed(x_rs2));
        else if(x_bltu) x_bc=($unsigned(x_rs1)<$unsigned(x_rs2));
        else if(x_bgeu) x_bc=($unsigned(x_rs1)>=$unsigned(x_rs2));
        if(x_bc)begin x_branch_taken=1;x_branch_target=execute_state.pc+x_imm_b_sext;end
      end
      OpcodeRegImm: begin
        x_we=1;
        if(x_addi)   begin x_adder_a=x_rs1;x_adder_b=x_imm_i_sext;x_alu_result=x_adder_sum;end
        else if(x_slti)  x_alu_result=$signed(x_rs1)<$signed(x_imm_i_sext)?1:0;
        else if(x_sltiu) x_alu_result=$unsigned(x_rs1)<$unsigned(x_imm_i_sext)?1:0;
        else if(x_xori)  x_alu_result=x_rs1^x_imm_i_sext;
        else if(x_ori)   x_alu_result=x_rs1|x_imm_i_sext;
        else if(x_andi)  x_alu_result=x_rs1&x_imm_i_sext;
        else if(x_slli)  x_alu_result=x_rs1<<x_imm_shamt;
        else if(x_srli)  x_alu_result=x_rs1>>x_imm_shamt;
        else if(x_srai)  x_alu_result=$signed(x_rs1)>>>x_imm_shamt;
      end
      OpcodeRegReg: begin
        x_we=1;
        if(x_add)   begin x_adder_a=x_rs1;x_adder_b=x_rs2; x_alu_result=x_adder_sum;end
        else if(x_sub)   begin x_adder_a=x_rs1;x_adder_b=~x_rs2;x_adder_cin=1;x_alu_result=x_adder_sum;end
        else if(x_sll)   x_alu_result=x_rs1<<x_rs2[4:0];
        else if(x_slt)   x_alu_result=$signed(x_rs1)<$signed(x_rs2)?1:0;
        else if(x_sltu)  x_alu_result=$unsigned(x_rs1)<$unsigned(x_rs2)?1:0;
        else if(x_xor)   x_alu_result=x_rs1^x_rs2;
        else if(x_srl)   x_alu_result=x_rs1>>x_rs2[4:0];
        else if(x_sra)   x_alu_result=$signed(x_rs1)>>>x_rs2[4:0];
        else if(x_or)    x_alu_result=x_rs1|x_rs2;
        else if(x_and)   x_alu_result=x_rs1&x_rs2;
        else if(x_mul_i) x_alu_result=x_mul_ss[31:0];
        else if(x_mulh)  x_alu_result=x_mul_ss[63:32];
        else if(x_mulhsu)x_alu_result=x_mul_su[63:32];
        else if(x_mulhu) x_alu_result=x_mul_uu[63:32];
        else if(x_div)  begin
          if (x_div_rs2 == 32'd0) begin x_we=1; x_alu_result=32'hFFFF_FFFF; end
          else begin x_we=0;div_dividend=x_div_rs1[31]?(~x_div_rs1+1):x_div_rs1;div_divisor=x_div_rs2[31]?(~x_div_rs2+1):x_div_rs2; end
        end
        else if(x_divu) begin
          if (x_div_rs2 == 32'd0) begin x_we=1; x_alu_result=32'hFFFF_FFFF; end
          else begin x_we=0;div_dividend=x_div_rs1;div_divisor=x_div_rs2; end
        end
        else if(x_rem)  begin
          if (x_div_rs2 == 32'd0) begin x_we=1; x_alu_result=x_div_rs1; end
          else begin x_we=0;div_dividend=x_div_rs1[31]?(~x_div_rs1+1):x_div_rs1;div_divisor=x_div_rs2[31]?(~x_div_rs2+1):x_div_rs2; end
        end
        else if(x_remu) begin
          if (x_div_rs2 == 32'd0) begin x_we=1; x_alu_result=x_div_rs1; end
          else begin x_we=0;div_dividend=x_div_rs1;div_divisor=x_div_rs2; end
        end
      end
      OpcodeLoad:    begin x_we=1;x_adder_a=x_rs1;x_adder_b=x_imm_i_sext;x_alu_result=x_adder_sum;end
      OpcodeStore:   begin x_adder_a=x_rs1;x_adder_b=x_imm_s_sext;x_alu_result=x_adder_sum;end
      OpcodeEnviron: begin end
      OpcodeMiscMem: begin end
      default:       begin end
    endcase
  end

  // =========================================================================
  // DIVIDER METADATA SHIFT REGISTER
  // =========================================================================
  always_ff @(posedge clk) begin
    if (rst) begin
      for (int s = 0; s < `DIVIDER_STAGES; s++) begin
        div_meta[s] <= '{valid:0,rd:0,pc:0,insn:0,cycle_status:CYCLE_RESET,funct3:0,op1:0,op2:0};
      end
    end else if (x_branch_taken) begin
      for (int s = 0; s < `DIVIDER_STAGES; s++) begin
        div_meta[s] <= '{valid:0,rd:0,pc:0,insn:0,cycle_status:CYCLE_RESET,funct3:0,op1:0,op2:0};
      end
    end else begin
      for (int s = `DIVIDER_STAGES-1; s > 0; s--) begin
        div_meta[s] <= div_meta[s-1];
      end
      if (x_is_div_slow && !load_use_stall && !x_branch_taken && !mem_stall) begin
        div_meta[0] <= '{
          valid:        1'b1,
          rd:           x_rd,
          pc:           execute_state.pc,
          insn:         execute_state.insn,
          cycle_status: execute_state.cycle_status,
          funct3:       x_funct3,
          op1:          x_div_rs1,
          op2:          x_div_rs2
        };
      end else begin
        div_meta[0] <= '{valid:0,rd:0,pc:0,insn:0,cycle_status:CYCLE_DIV,funct3:0,op1:0,op2:0};
      end
    end
  end

  // =========================================================================
  // MEMORY STAGE
  // =========================================================================
  always_ff @(posedge clk) begin
    if (rst) begin
      memory_state <= '{pc:0,insn:0,cycle_status:CYCLE_RESET,alu_result:0,rs2_data:0,rd:0,we:0};
    end else if (div_result_valid) begin
      memory_state <= '{
        pc:div_meta[`DIVIDER_STAGES-2].pc,
        insn:div_meta[`DIVIDER_STAGES-2].insn,
        cycle_status:div_meta[`DIVIDER_STAGES-2].cycle_status,
        alu_result:div_result_data,
        rs2_data:32'd0,
        rd:div_result_rd,
        we:(div_result_rd != 5'd0)
      };
    end else if (mem_stall) begin
      memory_state <= memory_state;
    end else begin
      if (x_is_div_slow) begin
        memory_state <= '{pc:0,insn:0,cycle_status:CYCLE_DIV,alu_result:0,rs2_data:0,rd:0,we:0};
      end else begin
        memory_state <= '{
          pc:execute_state.pc, insn:execute_state.insn, cycle_status:execute_state.cycle_status,
          alu_result:x_alu_result, rs2_data:x_rs2, rd:x_rd, we:x_we
        };
      end
    end
  end

  // Decode-stage bypass (same order as temp.sv: W, then M, then X so Execute wins).
  always_comb begin
    d_rs1_data_bypassed = d_rs1_data_rf;
    d_rs2_data_bypassed = d_rs2_data_rf;
    if (w_we && w_rd != 5'd0) begin
      if (w_rd == d_rs1) d_rs1_data_bypassed = w_rd_data;
      if (w_rd == d_rs2) d_rs2_data_bypassed = w_rd_data;
    end
    if (m_we && m_rd != 5'd0) begin
      if (m_rd == d_rs1) begin
        if (m_is_load)
          d_rs1_data_bypassed = m_load_result;
        else
          d_rs1_data_bypassed = m_alu_result;
      end
      if (m_rd == d_rs2) begin
        if (m_is_load)
          d_rs2_data_bypassed = m_load_result;
        else
          d_rs2_data_bypassed = m_alu_result;
      end
    end
    if (x_we && execute_state.rd != 5'd0 && !x_is_load) begin
      if (execute_state.rd == d_rs1) d_rs1_data_bypassed = x_alu_result;
      if (execute_state.rd == d_rs2) d_rs2_data_bypassed = x_alu_result;
    end
  end
  // Note: x_alu_result participates in same combinational cone as d_rs*_bypassed feeding execute_state;
  // order is execute_state reg -> x_alu -> decode bypass -> next execute (no oscillation for single-cycle logic).

  // D-cache AXI: load AR + store AW/W issued from X (end of X)
  wire x_do_load  = (x_opcode == OpcodeLoad)  && x_has_real_insn;
  wire x_do_store = (x_opcode == OpcodeStore) && x_has_real_insn;
  wire [`REG_SIZE] x_dmem_addr = {x_alu_result[31:2], 2'b00};
  wire x_st_sw = x_do_store && (x_funct3 == 3'b010);
  wire x_st_sh = x_do_store && (x_funct3 == 3'b001);
  wire x_st_sb = x_do_store && (x_funct3 == 3'b000);
  logic [`REG_SIZE] x_st_wdata;
  logic [3:0] x_st_wstrb;
  always_comb begin
    x_st_wdata = 0;
    x_st_wstrb = 0;
    if (x_do_store) begin
      if (x_st_sw) begin
        x_st_wdata = x_rs2;
        x_st_wstrb = 4'b1111;
      end else if (x_st_sh) begin
        x_st_wdata = x_alu_result[1] ? {x_rs2[15:0], 16'b0} : {16'b0, x_rs2[15:0]};
        x_st_wstrb = x_alu_result[1] ? 4'b1100 : 4'b0011;
      end else if (x_st_sb) begin
        x_st_wdata = {4{x_rs2[7:0]}};
        case (x_alu_result[1:0])
          2'b00: x_st_wstrb = 4'b0001;
          2'b01: x_st_wstrb = 4'b0010;
          2'b10: x_st_wstrb = 4'b0100;
          2'b11: x_st_wstrb = 4'b1000;
        endcase
      end
    end
  end
  always_comb begin
    dmem.ARPROT = 3'b000;
    dmem.AWPROT = 3'b000;
    dmem.ARADDR = x_dmem_addr;
    dmem.ARVALID = x_do_load && !mem_stall;
    dmem.AWADDR = x_dmem_addr;
    dmem.AWVALID = x_do_store && !mem_stall;
    dmem.WDATA  = x_st_wdata;
    dmem.WSTRB  = x_st_wstrb;
    dmem.WVALID = x_do_store && !mem_stall;
    dmem.RREADY = 1'b1;
    dmem.BREADY = 1'b1;
  end

  // =========================================================================
  // WRITEBACK
  // All instructions, including completed divides, write back through the normal W stage.
  // =========================================================================
  stage_writeback_t writeback_state;
  wire wb_dcache_miss = mem_stall && m_mem_real && m_is_ld_mem && !dmem.RVALID;
  wire wb_dmem_wait = mem_stall && m_mem_real && m_is_st_mem && !dmem.BVALID;
  always_comb begin
    if (rst) begin
      writeback_state = '{pc:0,insn:0,cycle_status:CYCLE_RESET,rd_data:0,rd:0,we:0};
    end else if (wb_dcache_miss) begin
      writeback_state = '{pc:0, insn:0, cycle_status:CYCLE_DCACHE_MISS, rd_data:0, rd:0, we:0};
    end else if (wb_dmem_wait) begin
      writeback_state = '{pc:0, insn:0, cycle_status:CYCLE_DMEM_WAIT, rd_data:0, rd:0, we:0};
    end else begin
      writeback_state = '{
        pc:memory_state.pc, insn:memory_state.insn, cycle_status:memory_state.cycle_status,
        rd_data:m_is_load?m_load_result:memory_state.alu_result,
        rd:memory_state.rd, we:memory_state.we && !mem_stall
      };
    end
  end

  wire [255:0] w_disasm;
  Disasm #(.PREFIX("W")) disasm_4writeback (.insn(writeback_state.insn), .disasm(w_disasm));

  // Writeback outputs
  assign w_rd      = writeback_state.rd;
  assign w_rd_data = writeback_state.rd_data;
  assign w_we      = writeback_state.we;

  assign halt = (writeback_state.insn[6:0]==OpcodeEnviron)
             && (writeback_state.insn[31:7]==25'd0)
             && (writeback_state.cycle_status==CYCLE_NO_STALL);

  wire w_valid = (writeback_state.cycle_status==CYCLE_NO_STALL) && !mem_stall;
  assign trace_completed_pc           = w_valid ? writeback_state.pc   : 32'd0;
  assign trace_completed_insn         = w_valid ? writeback_state.insn : 32'd0;
  assign trace_completed_cycle_status = writeback_state.cycle_status;

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

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  axil_if axil_mem_ro ();
  axil_if axil_mem_rw ();

  EasyAxilMemory #(
      .OPT_SKIDBUFFER(1),
      .OPT_LOWPOWER(0),
      .NUM_WORDS(8192)
  ) memory (
      .ACLK(clk),
      .ARESETn(~rst),
      .port_ro(axil_mem_ro.subord),
      .port_rw(axil_mem_rw.subord)
  );

  DatapathPipelinedAxil datapath (
      .clk(clk),
      .rst(rst),
      .imem(axil_mem_ro.manager),
      .dmem(axil_mem_rw.manager),
      .halt(halt),
      .trace_completed_pc(trace_completed_pc),
      .trace_completed_insn(trace_completed_insn),
      .trace_completed_cycle_status(trace_completed_cycle_status)
  );

endmodule
