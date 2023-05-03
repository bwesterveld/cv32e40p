// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Renzo Andri - andrire@student.ethz.ch                      //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                 Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    Execute stage                                              //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Execution stage: Hosts ALU and MAC unit                    //
//                 ALU: computes additions/subtractions/comparisons           //
//                 MULT: computes normal multiplications                      //
//                 APU_DISP: offloads instructions to the shared unit.        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e40p_ex_stage import cv32e40p_pkg::*; import cv32e40p_apu_core_pkg::*; import cv32e40p_fpu_pkg::*;
#(
    parameter FPU              = 0,
    parameter APU_NARGS_CPU    = 3,
    parameter APU_WOP_CPU      = 6,
    parameter APU_NDSFLAGS_CPU = 15,
    parameter APU_NUSFLAGS_CPU = 5
) (
    input logic clk,
    input logic rst_n,

    // ALU signals from ID stage
    input alu_opcode_e        alu_operator_i,
    input logic        [31:0] alu_operand_a_i,
    input logic        [31:0] alu_operand_b_i,
    input logic        [31:0] alu_operand_c_i,
    input logic               alu_en_i,
    input logic        [ 4:0] bmask_a_i,
    input logic        [ 4:0] bmask_b_i,
    input logic        [ 1:0] imm_vec_ext_i,
    input logic        [ 1:0] alu_vec_mode_i,
    input logic               alu_is_clpx_i,
    input logic               alu_is_subrot_i,
    input logic        [ 1:0] alu_clpx_shift_i,

    // Multiplier signals
    input mul_opcode_e        mult_operator_i,
    input logic        [31:0] mult_operand_a_i,
    input logic        [31:0] mult_operand_b_i,
    input logic        [31:0] mult_operand_c_i,
    input logic               mult_en_i,
    input logic               mult_sel_subword_i,
    input logic        [ 1:0] mult_signed_mode_i,
    input logic        [ 4:0] mult_imm_i,

    input logic [31:0] mult_dot_op_a_i,
    input logic [31:0] mult_dot_op_b_i,
    input logic [31:0] mult_dot_op_c_i,
    input logic [ 1:0] mult_dot_signed_i,
    input logic        mult_is_clpx_i,
    input logic [ 1:0] mult_clpx_shift_i,
    input logic        mult_clpx_img_i,

    output logic mult_multicycle_o,

    // FPU signals
    output logic fpu_fflags_we_o,

    // APU signals
    input logic                              apu_en_i,
    input logic [     APU_WOP_CPU-1:0]       apu_op_i,
    input logic [                 1:0]       apu_lat_i,
    input logic [   APU_NARGS_CPU-1:0][31:0] apu_operands_i,
    input logic [                 5:0]       apu_waddr_i,
    input logic [APU_NDSFLAGS_CPU-1:0]       apu_flags_i,

    input  logic [2:0][5:0] apu_read_regs_i,
    input  logic [2:0]      apu_read_regs_valid_i,
    output logic            apu_read_dep_o,
    input  logic [1:0][5:0] apu_write_regs_i,
    input  logic [1:0]      apu_write_regs_valid_i,
    output logic            apu_write_dep_o,

    output logic apu_perf_type_o,
    output logic apu_perf_cont_o,
    output logic apu_perf_wb_o,

    output logic apu_busy_o,
    output logic apu_ready_wb_o,

    // apu-interconnect
    // handshake signals
    output logic                           apu_req_o,
    input  logic                           apu_gnt_i,
    // request channel
    output logic [APU_NARGS_CPU-1:0][31:0] apu_operands_o,
    output logic [  APU_WOP_CPU-1:0]       apu_op_o,
    // response channel
    input  logic                           apu_rvalid_i,
    input  logic [             31:0]       apu_result_i,

    input logic        lsu_en_i,
    input logic [31:0] lsu_rdata_i,

    // input from ID stage
    input logic       branch_in_ex_i,
    input logic [5:0] regfile_alu_waddr_i,
    input logic       regfile_alu_we_i,

    // directly passed through to WB stage, not used in EX
    input logic       regfile_we_i,
    input logic [5:0] regfile_waddr_i,

    // CSR access
    input logic        csr_access_i,
    input logic [31:0] csr_rdata_i,

    // Output of EX stage pipeline
    output logic [ 5:0] regfile_waddr_wb_o,
    output logic        regfile_we_wb_o,
    output logic [31:0] regfile_wdata_wb_o,

    // Forwarding ports : to ID stage
    output logic [ 5:0] regfile_alu_waddr_fw_o,
    output logic        regfile_alu_we_fw_o,
    output logic [31:0] regfile_alu_wdata_fw_o,  // forward to RF and ID/EX pipe, ALU & MUL

    // To IF: Jump and branch target and decision
    output logic [31:0] jump_target_o,
    output logic        branch_decision_o,

    // Stall Control
    input logic         is_decoding_i, // Used to mask data Dependency inside the APU dispatcher in case of an istruction non valid
    input logic lsu_ready_ex_i,  // EX part of LSU is done
    input logic lsu_err_i,

    output logic ex_ready_o,  // EX stage ready for new data
    output logic ex_valid_o,  // EX stage gets new data
    input  logic wb_ready_i,  // WB stage ready for new data

    // Custom countermeasure signals
    input logic [31:0] cstm_instr_data_i,
    output logic [31:0] cstm_instr_data_o,

    input logic [2:0]  cstm_alu_op_a_mux_sel_i,      // operand a selection: reg value, PC, immediate or zero
    input logic [2:0]  cstm_alu_op_b_mux_sel_i,      // operand b selection: reg value or immediate
    input logic [1:0]  cstm_alu_op_c_mux_sel_i,      // operand c selection: reg value or jump target
    input logic [1:0]  cstm_alu_vec_mode_i,          // selects between 32 bit, 16 bit and 8 bit vectorial modes
    input logic        cstm_scalar_replication_i,    // scalar replication enable
    input logic        cstm_scalar_replication_c_i,  // scalar replication enable for operand C
    input logic [0:0]  cstm_imm_a_mux_sel_i,         // immediate selection for operand a
    input logic [3:0]  cstm_imm_b_mux_sel_i,         // immediate selection for operand b
    input logic [1:0]  cstm_regc_mux_i,              // register c selection: S3, RD or 0

    output alu_opcode_e cstm_alu_operator_o,
    output logic        cstm_alu_en_o,
    output logic        cstm_regfile_alu_we_o,
    output logic [2:0]  cstm_alu_op_a_mux_sel_o,      // operand a selection: reg value, PC, immediate or zero
    output logic [2:0]  cstm_alu_op_b_mux_sel_o,      // operand b selection: reg value or immediate
    output logic [1:0]  cstm_alu_op_c_mux_sel_o,      // operand c selection: reg value or jump target
    output logic [1:0]  cstm_alu_vec_mode_o,          // selects between 32 bit, 16 bit and 8 bit vectorial modes
    output logic        cstm_scalar_replication_o,    // scalar replication enable
    output logic        cstm_scalar_replication_c_o,  // scalar replication enable for operand C
    output logic [0:0]  cstm_imm_a_mux_sel_o,         // immediate selection for operand a
    output logic [3:0]  cstm_imm_b_mux_sel_o,         // immediate selection for operand b
    output logic [1:0]  cstm_regc_mux_o,              // register c selection: S3, RD or 0

    // ALU RV32IM Multiplication extension
    input logic        cstm_rega_used_i,             // rs1 is used by current instruction
    input logic        cstm_regb_used_i,             // rs2 is used by current instruction
    input logic        cstm_regc_used_i,             // rs3 is used by current instruction
    input mul_opcode_e cstm_mult_operator_i,         // Multiplication operation selection
    input logic        cstm_mult_int_en_i,           // perform integer multiplication
    input logic [0:0]  cstm_mult_imm_mux_i,          // Multiplication immediate mux selector
    input logic [1:0]  cstm_mult_signed_mode_i,      // Multiplication in signed mode

    output logic        cstm_rega_used_o,             // rs1 is used by current instruction
    output logic        cstm_regb_used_o,             // rs2 is used by current instruction
    output logic        cstm_regc_used_o,             // rs3 is used by current instruction
    output mul_opcode_e cstm_mult_operator_o,         // Multiplication operation selection
    output logic        cstm_mult_int_en_o,           // perform integer multiplication
    output logic [0:0]  cstm_mult_imm_mux_o,          // Multiplication immediate mux selector
    output logic [1:0]  cstm_mult_signed_mode_o      // Multiplication in signed mode

);

  logic [31:0] alu_result;
  logic [31:0] mult_result;
  logic        alu_cmp_result;

  logic        regfile_we_lsu;
  logic [ 5:0] regfile_waddr_lsu;

  logic        wb_contention;
  logic        wb_contention_lsu;

  logic        alu_ready;
  logic        mult_ready;

  // APU signals
  logic        apu_valid;
  logic [ 5:0] apu_waddr;
  logic [31:0] apu_result;
  logic        apu_stall;
  logic        apu_active;
  logic        apu_singlecycle;
  logic        apu_multicycle;
  logic        apu_req;
  logic        apu_gnt;

  // ALU write port mux
  always_comb begin
    regfile_alu_wdata_fw_o = '0;
    regfile_alu_waddr_fw_o = '0;
    regfile_alu_we_fw_o    = '0;
    wb_contention          = 1'b0;

    // APU single cycle operations, and multicycle operations (>2cycles) are written back on ALU port
    if (apu_valid & (apu_singlecycle | apu_multicycle)) begin
      regfile_alu_we_fw_o    = 1'b1;
      regfile_alu_waddr_fw_o = apu_waddr;
      regfile_alu_wdata_fw_o = apu_result;

      if (regfile_alu_we_i & ~apu_en_i) begin
        wb_contention = 1'b1;
      end
    end else begin
      regfile_alu_we_fw_o    = regfile_alu_we_i & ~apu_en_i;  // private fpu incomplete?
      regfile_alu_waddr_fw_o = regfile_alu_waddr_i;
      if (alu_en_i) regfile_alu_wdata_fw_o = alu_result;
      if (mult_en_i) regfile_alu_wdata_fw_o = mult_result;
      if (csr_access_i) regfile_alu_wdata_fw_o = csr_rdata_i;
    end
  end

  // LSU write port mux
  always_comb begin
    regfile_we_wb_o    = 1'b0;
    regfile_waddr_wb_o = regfile_waddr_lsu;
    regfile_wdata_wb_o = lsu_rdata_i;
    wb_contention_lsu  = 1'b0;

    if (regfile_we_lsu) begin
      regfile_we_wb_o = 1'b1;
      if (apu_valid & (!apu_singlecycle & !apu_multicycle)) begin
        wb_contention_lsu = 1'b1;
      end
      // APU two-cycle operations are written back on LSU port
    end else if (apu_valid & (!apu_singlecycle & !apu_multicycle)) begin
      regfile_we_wb_o    = 1'b1;
      regfile_waddr_wb_o = apu_waddr;
      regfile_wdata_wb_o = apu_result;
    end
  end

  // branch handling
  assign branch_decision_o = alu_cmp_result;
  assign jump_target_o     = alu_operand_c_i;


  ////////////////////////////
  //     _    _    _   _    //
  //    / \  | |  | | | |   //
  //   / _ \ | |  | | | |   //
  //  / ___ \| |__| |_| |   //
  // /_/   \_\_____\___/    //
  //                        //
  ////////////////////////////

  cv32e40p_alu alu_i (
      .clk        (clk),
      .rst_n      (rst_n),
      .enable_i   (alu_en_i),
      .operator_i (alu_operator_i),
      .operand_a_i(alu_operand_a_i),
      .operand_b_i(alu_operand_b_i),
      .operand_c_i(alu_operand_c_i),

      .vector_mode_i(alu_vec_mode_i),
      .bmask_a_i    (bmask_a_i),
      .bmask_b_i    (bmask_b_i),
      .imm_vec_ext_i(imm_vec_ext_i),

      .is_clpx_i   (alu_is_clpx_i),
      .clpx_shift_i(alu_clpx_shift_i),
      .is_subrot_i (alu_is_subrot_i),

      .result_o           (alu_result),
      .comparison_result_o(alu_cmp_result),

      .ready_o   (alu_ready),
      .ex_ready_i(ex_ready_o)
  );


  ////////////////////////////////////////////////////////////////
  //  __  __ _   _ _   _____ ___ ____  _     ___ _____ ____     //
  // |  \/  | | | | | |_   _|_ _|  _ \| |   |_ _| ____|  _ \    //
  // | |\/| | | | | |   | |  | || |_) | |    | ||  _| | |_) |   //
  // | |  | | |_| | |___| |  | ||  __/| |___ | || |___|  _ <    //
  // |_|  |_|\___/|_____|_| |___|_|   |_____|___|_____|_| \_\   //
  //                                                            //
  ////////////////////////////////////////////////////////////////

  cv32e40p_mult mult_i (
      .clk  (clk),
      .rst_n(rst_n),

      .enable_i  (mult_en_i),
      .operator_i(mult_operator_i),

      .short_subword_i(mult_sel_subword_i),
      .short_signed_i (mult_signed_mode_i),

      .op_a_i(mult_operand_a_i),
      .op_b_i(mult_operand_b_i),
      .op_c_i(mult_operand_c_i),
      .imm_i (mult_imm_i),

      .dot_op_a_i  (mult_dot_op_a_i),
      .dot_op_b_i  (mult_dot_op_b_i),
      .dot_op_c_i  (mult_dot_op_c_i),
      .dot_signed_i(mult_dot_signed_i),
      .is_clpx_i   (mult_is_clpx_i),
      .clpx_shift_i(mult_clpx_shift_i),
      .clpx_img_i  (mult_clpx_img_i),

      .result_o(mult_result),

      .multicycle_o(mult_multicycle_o),
      .ready_o     (mult_ready),
      .ex_ready_i  (ex_ready_o)
  );

  generate
    ////////////////////////////////////////////////////
    //     _    ____  _   _   ____ ___ ____  ____     //
    //    / \  |  _ \| | | | |  _ \_ _/ ___||  _ \    //
    //   / _ \ | |_) | | | | | | | | |\___ \| |_) |   //
    //  / ___ \|  __/| |_| | | |_| | | ___) |  __/    //
    // /_/   \_\_|    \___/  |____/___|____/|_|       //
    //                                                //
    ////////////////////////////////////////////////////
    begin : gen_no_apu
      // default assignements for the case when no FPU/APU is attached.
      assign apu_req_o         = '0;
      assign apu_operands_o[0] = '0;
      assign apu_operands_o[1] = '0;
      assign apu_operands_o[2] = '0;
      assign apu_op_o          = '0;
      assign apu_req           = 1'b0;
      assign apu_gnt           = 1'b0;
      assign apu_result        = 32'b0;
      assign apu_valid         = 1'b0;
      assign apu_waddr         = 6'b0;
      assign apu_stall         = 1'b0;
      assign apu_active        = 1'b0;
      assign apu_ready_wb_o    = 1'b1;
      assign apu_perf_wb_o     = 1'b0;
      assign apu_perf_cont_o   = 1'b0;
      assign apu_perf_type_o   = 1'b0;
      assign apu_singlecycle   = 1'b0;
      assign apu_multicycle    = 1'b0;
      assign apu_read_dep_o    = 1'b0;
      assign apu_write_dep_o   = 1'b0;
      assign fpu_fflags_we_o   = 1'b0;

    end
  endgenerate

  assign apu_busy_o = apu_active;

  ///////////////////////////////////////
  // EX/WB Pipeline Register           //
  ///////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n) begin : EX_WB_Pipeline_Register
    if (~rst_n) begin
      regfile_waddr_lsu <= '0;
      regfile_we_lsu    <= 1'b0;

      // Forward instruction
      cstm_instr_data_o <= 32'b0;

      // Forward ALU signals
      cstm_alu_en_o                    = 1'b1;
      cstm_alu_operator_o              = ALU_SLTU;
      cstm_alu_op_a_mux_sel_o          = OP_A_REGA_OR_FWD;
      cstm_alu_op_b_mux_sel_o          = OP_B_REGB_OR_FWD;
      cstm_alu_op_c_mux_sel_o          = OP_C_REGC_OR_FWD;
      cstm_alu_vec_mode_o              = VEC_MODE32;
      cstm_scalar_replication_o        = 1'b0;
      cstm_scalar_replication_c_o      = 1'b0;
      cstm_regc_mux_o                  = REGC_ZERO;
      cstm_imm_a_mux_sel_o             = IMMA_ZERO;
      cstm_imm_b_mux_sel_o             = IMMB_I;

      cstm_rega_used_o                 = 1'b0;
      cstm_regb_used_o                 = 1'b0;
      cstm_regc_used_o                 = 1'b0;
      cstm_mult_operator_o             = MUL_I;
      cstm_mult_int_en                 = 1'b0;
      cstm_mult_imm_mux_o              = MIMM_ZERO;
      cstm_mult_signed_mode_o          = 2'b00;


    end else begin
      if (ex_valid_o) // wb_ready_i is implied
      begin
        regfile_we_lsu <= regfile_we_i & ~lsu_err_i;
        if (regfile_we_i & ~lsu_err_i) begin
          regfile_waddr_lsu <= regfile_waddr_i;
        end

        // These custom control signals are just forwarded.
        // Instruction
        cstm_instr_data_o <= cstm_instr_data_i;

        // ALU
        cstm_alu_operator_o <= alu_operator_i;
        cstm_alu_en_o <= alu_en_i;
        cstm_regfile_alu_we_o <= regfile_alu_we_i;
        cstm_alu_op_a_mux_sel_o <= cstm_alu_op_a_mux_sel_i;
        cstm_alu_op_b_mux_sel_o <= cstm_alu_op_b_mux_sel_i;
        cstm_alu_op_c_mux_sel_o <= cstm_alu_op_c_mux_sel_i;
        cstm_alu_vec_mode_o <= cstm_alu_vec_mode_i;
        cstm_scalar_replication_o <= cstm_scalar_replication_i;
        cstm_scalar_replication_c_o <= cstm_scalar_replication_c_i;
        cstm_imm_a_mux_sel_o <= cstm_imm_a_mux_sel_i;
        cstm_imm_b_mux_sel_o <= cstm_imm_b_mux_sel_i;
        cstm_regc_mux_o <= cstm_regc_mux_i;

        // ALU Mult
        cstm_rega_used_o <= cstm_rega_used_o;
        cstm_regb_used_o <= cstm_regb_used_o;
        cstm_regc_used_o <= cstm_regc_used_o;
        cstm_mult_operator_o <= cstm_mult_operator_o;
        cstm_mult_int_en_o <= cstm_mult_int_en_o;
        cstm_mult_imm_mux_o <= cstm_mult_imm_mux_o;
        cstm_mult_signed_mode_o <= cstm_mult_signed_mode_o;

      end else if (wb_ready_i) begin
        // we are ready for a new instruction, but there is none available,
        // so we just flush the current one out of the pipe
        regfile_we_lsu <= 1'b0;
        cstm_instr_data_o <= 32'b0;
      end
    end
  end

  // As valid always goes to the right and ready to the left, and we are able
  // to finish branches without going to the WB stage, ex_valid does not
  // depend on ex_ready.
  assign ex_ready_o = (~apu_stall & alu_ready & mult_ready & lsu_ready_ex_i
                       & wb_ready_i & ~wb_contention) | (branch_in_ex_i);
  assign ex_valid_o = (apu_valid | alu_en_i | mult_en_i | csr_access_i | lsu_en_i)
                       & (alu_ready & mult_ready & lsu_ready_ex_i & wb_ready_i);

endmodule
