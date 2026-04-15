////////////////////////////////////////////////
// Generative AI was used to make this code.
////////////////////////////////////////////////
////////////////////////////////////////////////
// Generative AI was used to make this code.
////////////////////////////////////////////////
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
  assign regs[0]  = 32'd0;
  assign rs1_data = (we && rd != 5'd0 && rd == rs1) ? rd_data : regs[rs1];
  assign rs2_data = (we && rd != 5'd0 && rd == rs2) ? rd_data : regs[rs2];
  assign regs[0]  = 32'd0;
  assign rs1_data = (we && rd != 5'd0 && rd == rs1) ? rd_data : regs[rs1];
  assign rs2_data = (we && rd != 5'd0 && rd == rs2) ? rd_data : regs[rs2];
  genvar i;
  for (i = 1; i < 32; i = i + 1) begin : gen_regs
    always_ff @(posedge clk) begin
      if (rst) regs[i] <= 32'd0;
      else if (we && rd == i) regs[i] <= rd_data;
    end
  end
endmodule

// ---------------------------------------------------------------------------
// Stage structs
// ---------------------------------------------------------------------------
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
  logic [`REG_SIZE]  pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e     cycle_status;
  logic [`REG_SIZE]  rs1_data;
  logic [`REG_SIZE]  rs2_data;
  logic [4:0]        rs1;
  logic [4:0]        rs2;
  logic [4:0]        rd;
  cycle_status_e     cycle_status;
  logic [`REG_SIZE]  rs1_data;
  logic [`REG_SIZE]  rs2_data;
  logic [4:0]        rs1;
  logic [4:0]        rs2;
  logic [4:0]        rd;
} stage_execute_t;

typedef struct packed {
  logic [`REG_SIZE]  pc;
  logic [`REG_SIZE]  pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e     cycle_status;
  logic [`REG_SIZE]  alu_result;
  logic [`REG_SIZE]  rs2_data;
  logic [4:0]        rd;
  logic              we;
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
  logic             valid;    // 1 = a divide result is emerging this stage
  logic [4:0]       rd;       // destination register
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e    cycle_status;
  logic [2:0]       funct3;   // encodes div/divu/rem/remu
  logic [`REG_SIZE] op1;      // original rs1 (for sign/zero correction)
  logic [`REG_SIZE] op2;      // original rs2 (for sign/zero correction)
} div_meta_t;

// ---------------------------------------------------------------------------
// DatapathPipelined
// ---------------------------------------------------------------------------
  cycle_status_e    cycle_status;
  logic [2:0]       funct3;   // encodes div/divu/rem/remu
  logic [`REG_SIZE] op1;      // original rs1 (for sign/zero correction)
  logic [`REG_SIZE] op2;      // original rs2 (for sign/zero correction)
} div_meta_t;

// ---------------------------------------------------------------------------
// DatapathPipelined
// ---------------------------------------------------------------------------
module DatapathPipelined (
    input  wire              clk,
    input  wire              rst,
    input  wire              clk,
    input  wire              rst,
    output logic [`REG_SIZE] pc_to_imem,
    input  wire  [`REG_SIZE] insn_from_imem,
    input  wire  [`REG_SIZE] insn_from_imem,
    output logic [`REG_SIZE] addr_to_dmem,
    input  wire  [`REG_SIZE] load_data_from_dmem,
    input  wire  [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0]       store_we_to_dmem,
    output logic             halt,
    output logic [3:0]       store_we_to_dmem,
    output logic             halt,
    output logic [`REG_SIZE] trace_completed_pc,
    output logic [`INSN_SIZE] trace_completed_insn,
    output cycle_status_e    trace_completed_cycle_status
    output cycle_status_e    trace_completed_cycle_status
);

  localparam bit [`OPCODE_SIZE] OpcodeLoad    = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeStore   = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeBranch  = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeJalr    = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpcodeLoad    = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeStore   = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeBranch  = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeJalr    = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpcodeMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpcodeJal     = 7'b11_011_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegImm  = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegReg  = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeJal     = 7'b11_011_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegImm  = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegReg  = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeEnviron = 7'b11_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeAuipc   = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpcodeLui     = 7'b01_101_11;

  localparam bit [`OPCODE_SIZE] OpcodeAuipc   = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpcodeLui     = 7'b01_101_11;

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
  // Store-data deps (load->store rs2) are resolved by WM bypass; do not stall.
  wire d_needs_load_rs1 = d_uses_rs1 && (d_rs1 == execute_state.insn[11:7]);
  wire d_needs_load_rs2 = d_uses_rs2 && !d_is_store && (d_rs2 == execute_state.insn[11:7]);
  wire d_needs_load = d_needs_load_rs1 || d_needs_load_rs2;
  wire load_use_stall = x_is_load && x_load_rd_nz && d_needs_load
                     && (decode_state.cycle_status == CYCLE_NO_STALL);

  wire pipeline_stall = load_use_stall || div_stall || div_insert_stall || stall_div_wb;

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
  // FETCH
  // =========================================================================
  logic [`REG_SIZE] f_pc_current;
  wire  [`REG_SIZE] f_insn = insn_from_imem;
  cycle_status_e    f_cycle_status;
  wire  [`REG_SIZE] f_insn = insn_from_imem;
  cycle_status_e    f_cycle_status;

  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current   <= 32'd0;
      f_pc_current   <= 32'd0;
      f_cycle_status <= CYCLE_NO_STALL;
    end else begin
      f_cycle_status <= CYCLE_NO_STALL;
      if (pipeline_stall)      f_pc_current <= f_pc_current;
      else if (x_branch_taken) f_pc_current <= x_branch_target;
      else                     f_pc_current <= f_pc_current + 4;
      if (pipeline_stall)      f_pc_current <= f_pc_current;
      else if (x_branch_taken) f_pc_current <= x_branch_target;
      else                     f_pc_current <= f_pc_current + 4;
    end
  end
  assign pc_to_imem = f_pc_current;

  wire [255:0] f_disasm;
  Disasm #(.PREFIX("F")) disasm_0fetch (.insn(f_insn), .disasm(f_disasm));

  // =========================================================================
  // DECODE
  // =========================================================================
  // =========================================================================
  // DECODE
  // =========================================================================
  always_ff @(posedge clk) begin
    if (rst)
      decode_state <= '{pc:0, insn:0, cycle_status:CYCLE_RESET};
    else if (pipeline_stall)
      decode_state <= decode_state;
    else if (x_branch_taken)
      decode_state <= '{pc:0, insn:0, cycle_status:CYCLE_TAKEN_BRANCH};
    else
      decode_state <= '{pc:f_pc_current, insn:f_insn, cycle_status:f_cycle_status};
    if (rst)
      decode_state <= '{pc:0, insn:0, cycle_status:CYCLE_RESET};
    else if (pipeline_stall)
      decode_state <= decode_state;
    else if (x_branch_taken)
      decode_state <= '{pc:0, insn:0, cycle_status:CYCLE_TAKEN_BRANCH};
    else
      decode_state <= '{pc:f_pc_current, insn:f_insn, cycle_status:f_cycle_status};
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

  // MX/WX bypass
  wire mx_rs1 = m_we && m_rd!=5'd0 && m_rd==execute_state.rs1;
  wire mx_rs2 = m_we && m_rd!=5'd0 && m_rd==execute_state.rs2;
  wire wx_rs1 = w_we && w_rd!=5'd0 && w_rd==execute_state.rs1;
  wire wx_rs2 = w_we && w_rd!=5'd0 && w_rd==execute_state.rs2;
  // Memory is younger than Writeback: MX before WX (matches temp.sv intent).
  wire [`REG_SIZE] x_rs1 = mx_rs1 ? m_alu_result : wx_rs1 ? w_rd_data : execute_state.rs1_data;
  wire [`REG_SIZE] x_rs2 = mx_rs2 ? m_alu_result : wx_rs2 ? w_rd_data : execute_state.rs2_data;

  // M->X for div operands: completed divide sits in memory_state.alu_result (see M-stage mux).
  wire mx_div_rs1 = m_we && m_rd != 5'd0 && m_rd == execute_state.rs1;
  wire mx_div_rs2 = m_we && m_rd != 5'd0 && m_rd == execute_state.rs2;
  wire wx_div_rs1 = w_we && w_rd != 5'd0 && w_rd == execute_state.rs1;
  wire wx_div_rs2 = w_we && w_rd != 5'd0 && w_rd == execute_state.rs2;
  wire [`REG_SIZE] x_div_rs1 = mx_div_rs1 ? memory_state.alu_result
                                          : wx_div_rs1 ? w_rd_data
                                                       : execute_state.rs1_data;
  wire [`REG_SIZE] x_div_rs2 = mx_div_rs2 ? memory_state.alu_result
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
    // Dummy operands when no active div in X: divider advances each cycle; 1/1 avoids div-by-zero ambiguity.
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
        // Divides: feed divider combinationally; result via div_meta pipeline
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
      OpcodeEnviron: begin end  // ecall: halt detected in W
      OpcodeMiscMem: begin end  // fence: NOP, x_we=0
      default:       begin end
    endcase
  end

  // =========================================================================
  // DIVIDER METADATA SHIFT REGISTER
  // Shifts every cycle regardless of pipeline_stall (divider never stalls).
  // Stage 0 is loaded when a divide passes through Execute (non-stall cycle).
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
      if (x_is_div_slow && !load_use_stall && !x_branch_taken) begin
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
  stage_memory_t memory_state;
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

  wire wm_bypass    = w_we&&w_rd!=5'd0&&w_rd==memory_state.insn[24:20]&&m_is_store;
  wire [`REG_SIZE] m_store_data = wm_bypass ? w_rd_data : memory_state.rs2_data;

  logic [`REG_SIZE] m_load_result;
  always_comb begin
    m_load_result=0;
    if(m_lw) m_load_result=load_data_from_dmem;
    else if(m_lh)  m_load_result=memory_state.alu_result[1]?{{16{load_data_from_dmem[31]}},load_data_from_dmem[31:16]}:{{16{load_data_from_dmem[15]}},load_data_from_dmem[15:0]};
    else if(m_lhu) m_load_result=memory_state.alu_result[1]?{16'b0,load_data_from_dmem[31:16]}:{16'b0,load_data_from_dmem[15:0]};
    else if(m_lb) case(memory_state.alu_result[1:0])
      2'b00:m_load_result={{24{load_data_from_dmem[7]}},load_data_from_dmem[7:0]};
      2'b01:m_load_result={{24{load_data_from_dmem[15]}},load_data_from_dmem[15:8]};
      2'b10:m_load_result={{24{load_data_from_dmem[23]}},load_data_from_dmem[23:16]};
      2'b11:m_load_result={{24{load_data_from_dmem[31]}},load_data_from_dmem[31:24]};
    endcase
    else if(m_lbu) case(memory_state.alu_result[1:0])
      2'b00:m_load_result={24'b0,load_data_from_dmem[7:0]};
      2'b01:m_load_result={24'b0,load_data_from_dmem[15:8]};
      2'b10:m_load_result={24'b0,load_data_from_dmem[23:16]};
      2'b11:m_load_result={24'b0,load_data_from_dmem[31:24]};
    endcase
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

  always_comb begin
    addr_to_dmem=0;store_data_to_dmem=0;store_we_to_dmem=0;
    if(m_is_load) addr_to_dmem={memory_state.alu_result[31:2],2'b0};
    else if(m_is_store) begin
      addr_to_dmem={memory_state.alu_result[31:2],2'b0};
      if(m_sw) begin store_data_to_dmem=m_store_data;store_we_to_dmem=4'b1111;end
      else if(m_sh) begin
        store_data_to_dmem=memory_state.alu_result[1]?{m_store_data[15:0],16'b0}:{16'b0,m_store_data[15:0]};
        store_we_to_dmem=memory_state.alu_result[1]?4'b1100:4'b0011;
      end else if(m_sb) begin
        store_data_to_dmem={4{m_store_data[7:0]}};
        case(memory_state.alu_result[1:0])
          2'b00:store_we_to_dmem=4'b0001;2'b01:store_we_to_dmem=4'b0010;
          2'b10:store_we_to_dmem=4'b0100;2'b11:store_we_to_dmem=4'b1000;
        endcase
      end
    end
  end

  // =========================================================================
  // WRITEBACK
  // All instructions, including completed divides, write back through the normal W stage.
  // =========================================================================
  stage_writeback_t writeback_state;
    addr_to_dmem=0;store_data_to_dmem=0;store_we_to_dmem=0;
    if(m_is_load) addr_to_dmem={memory_state.alu_result[31:2],2'b0};
    else if(m_is_store) begin
      addr_to_dmem={memory_state.alu_result[31:2],2'b0};
      if(m_sw) begin store_data_to_dmem=m_store_data;store_we_to_dmem=4'b1111;end
      else if(m_sh) begin
        store_data_to_dmem=memory_state.alu_result[1]?{m_store_data[15:0],16'b0}:{16'b0,m_store_data[15:0]};
        store_we_to_dmem=memory_state.alu_result[1]?4'b1100:4'b0011;
      end else if(m_sb) begin
        store_data_to_dmem={4{m_store_data[7:0]}};
        case(memory_state.alu_result[1:0])
          2'b00:store_we_to_dmem=4'b0001;2'b01:store_we_to_dmem=4'b0010;
          2'b10:store_we_to_dmem=4'b0100;2'b11:store_we_to_dmem=4'b1000;
        endcase
      end
    end
  end

  // =========================================================================
  // WRITEBACK
  // All instructions, including completed divides, write back through the normal W stage.
  // =========================================================================
  stage_writeback_t writeback_state;
  always_ff @(posedge clk) begin
    if (rst)
      writeback_state <= '{pc:0,insn:0,cycle_status:CYCLE_RESET,rd_data:0,rd:0,we:0};
    else
    if (rst)
      writeback_state <= '{pc:0,insn:0,cycle_status:CYCLE_RESET,rd_data:0,rd:0,we:0};
    else
      writeback_state <= '{
        pc:memory_state.pc, insn:memory_state.insn, cycle_status:memory_state.cycle_status,
        rd_data:m_is_load?m_load_result:memory_state.alu_result,
        rd:memory_state.rd, we:memory_state.we
      };
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

  wire w_valid = writeback_state.cycle_status==CYCLE_NO_STALL;
  assign trace_completed_pc           = w_valid ? writeback_state.pc   : 32'd0;
  assign trace_completed_insn         = w_valid ? writeback_state.insn : 32'd0;
        pc:memory_state.pc, insn:memory_state.insn, cycle_status:memory_state.cycle_status,
        rd_data:m_is_load?m_load_result:memory_state.alu_result,
        rd:memory_state.rd, we:memory_state.we
      };
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

  wire w_valid = writeback_state.cycle_status==CYCLE_NO_STALL;
  assign trace_completed_pc           = w_valid ? writeback_state.pc   : 32'd0;
  assign trace_completed_insn         = w_valid ? writeback_state.insn : 32'd0;
  assign trace_completed_cycle_status = writeback_state.cycle_status;

endmodule

module MemorySingleCycle #(parameter int NUM_WORDS=512) (
    input  wire rst,clk,
    input  wire [`REG_SIZE]  pc_to_imem,
module MemorySingleCycle #(parameter int NUM_WORDS=512) (
    input  wire rst,clk,
    input  wire [`REG_SIZE]  pc_to_imem,
    output logic [`REG_SIZE] insn_from_imem,
    input  wire [`REG_SIZE]  addr_to_dmem,
    input  wire [`REG_SIZE]  addr_to_dmem,
    output logic [`REG_SIZE] load_data_from_dmem,
    input  wire [`REG_SIZE]  store_data_to_dmem,
    input  wire [3:0]        store_we_to_dmem
    input  wire [`REG_SIZE]  store_data_to_dmem,
    input  wire [3:0]        store_we_to_dmem
);
  logic [`REG_SIZE] mem_array[NUM_WORDS];
`ifdef SYNTHESIS
  initial begin $readmemh("mem_initial_contents.hex",mem_array); end
  initial begin $readmemh("mem_initial_contents.hex",mem_array); end
`endif
  always_comb begin assert(pc_to_imem[1:0]==2'b00);assert(addr_to_dmem[1:0]==2'b00);end
  localparam int AddrMsb=$clog2(NUM_WORDS)+1,AddrLsb=2;
  always @(negedge clk) if(!rst) insn_from_imem<=mem_array[pc_to_imem[AddrMsb:AddrLsb]];
  always @(negedge clk) if(!rst) begin
    if(store_we_to_dmem[0])mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0]  <=store_data_to_dmem[7:0];
    if(store_we_to_dmem[1])mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <=store_data_to_dmem[15:8];
    if(store_we_to_dmem[2])mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16]<=store_data_to_dmem[23:16];
    if(store_we_to_dmem[3])mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24]<=store_data_to_dmem[31:24];
    load_data_from_dmem<=mem_array[addr_to_dmem[AddrMsb:AddrLsb]];
  always_comb begin assert(pc_to_imem[1:0]==2'b00);assert(addr_to_dmem[1:0]==2'b00);end
  localparam int AddrMsb=$clog2(NUM_WORDS)+1,AddrLsb=2;
  always @(negedge clk) if(!rst) insn_from_imem<=mem_array[pc_to_imem[AddrMsb:AddrLsb]];
  always @(negedge clk) if(!rst) begin
    if(store_we_to_dmem[0])mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0]  <=store_data_to_dmem[7:0];
    if(store_we_to_dmem[1])mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <=store_data_to_dmem[15:8];
    if(store_we_to_dmem[2])mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16]<=store_data_to_dmem[23:16];
    if(store_we_to_dmem[3])mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24]<=store_data_to_dmem[31:24];
    load_data_from_dmem<=mem_array[addr_to_dmem[AddrMsb:AddrLsb]];
  end
endmodule

module Processor (
    input  wire  clk,rst,
    input  wire  clk,rst,
    output logic halt,
    output wire [`REG_SIZE]  trace_completed_pc,
    output wire [`REG_SIZE]  trace_completed_pc,
    output wire [`INSN_SIZE] trace_completed_insn,
    output cycle_status_e    trace_completed_cycle_status
    output cycle_status_e    trace_completed_cycle_status
);
  wire [`INSN_SIZE] insn_from_imem;
  wire [`REG_SIZE]  pc_to_imem,mem_data_addr,mem_data_loaded_value,mem_data_to_write;
  wire [`REG_SIZE]  pc_to_imem,mem_data_addr,mem_data_loaded_value,mem_data_to_write;
  wire [3:0] mem_data_we;
  wire [(8*32)-1:0] test_case;
  MemorySingleCycle #(.NUM_WORDS(8192)) memory (
      .rst(rst),.clk(clk),.pc_to_imem(pc_to_imem),.insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),.load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem(mem_data_to_write),.store_we_to_dmem(mem_data_we));
  MemorySingleCycle #(.NUM_WORDS(8192)) memory (
      .rst(rst),.clk(clk),.pc_to_imem(pc_to_imem),.insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),.load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem(mem_data_to_write),.store_we_to_dmem(mem_data_we));
  DatapathPipelined datapath (
      .clk(clk),.rst(rst),.pc_to_imem(pc_to_imem),.insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),.store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),.load_data_from_dmem(mem_data_loaded_value),
      .halt(halt),.trace_completed_pc(trace_completed_pc),
      .clk(clk),.rst(rst),.pc_to_imem(pc_to_imem),.insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),.store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),.load_data_from_dmem(mem_data_loaded_value),
      .halt(halt),.trace_completed_pc(trace_completed_pc),
      .trace_completed_insn(trace_completed_insn),
      .trace_completed_cycle_status(trace_completed_cycle_status));
      .trace_completed_cycle_status(trace_completed_cycle_status));
endmodule
