`timescale 1ns/100ps
`default_nettype none

/***** top module for simulation *****/
module m_top (); 
   reg r_clk=0; initial forever #50 r_clk = ~r_clk;
   wire [31:0] w_led;
   reg [31:0] r_cnt = 1;
   always@(posedge r_clk) r_cnt <= r_cnt + 1;

   m_proc06 p (r_clk, 1'b1, w_led);
   
   always@(posedge r_clk) $write("%7d %08x\n", r_cnt, p.w_rslt2_wb);
   always@(posedge r_clk) if(w_led!=0) $finish;
   initial #50000000 $finish;
endmodule

/*
module m_main (w_clk, w_led);
   input  wire w_clk;
   output wire [3:0] w_led;
 
   wire w_clk2, w_locked;
   clk_wiz_0 clk_w0 (w_clk2, 0, w_locked, w_clk);
   
   wire [31:0] w_dout;
   m_proc06 p (w_clk2, w_locked, w_dout);

   vio_0 vio_00(w_clk2, w_dout);
 
   reg [3:0] r_led = 0;
   always @(posedge w_clk2) 
     r_led <= {^w_dout[31:24], ^w_dout[23:16], ^w_dout[15:8], ^w_dout[7:0]};
   assign w_led = r_led;
endmodule
*/

module m_proc06 (w_clk, w_ce, w_led);
  input  wire w_clk, w_ce;
  output wire [31:0] w_led;

  reg [1:0] bht [0:63];
  integer i;
  initial begin
    for (i = 0; i < 64; i = i + 1) begin
        bht[i] = 2'b01;
    end
  end

/********************************* PIPELINE *********************************/
  reg [31:0] if_id_pc = 0, if_id_ir = 0;
  reg if_id_pred_taken = 0;
  
  reg [31:0] id_ex_pc = 0, id_ex_rrs1 = 0, id_ex_rrs2 = 0, id_ex_imm = 0;
  reg [4:0]  id_ex_rd = 0, id_ex_op5 = 0;
  reg [4:0]  id_ex_rs1 = 0, id_ex_rs2 = 0; 
  reg [2:0]  id_ex_funct3 = 0;
  reg        id_ex_we = 0, id_ex_mem_we = 0;
  reg        id_ex_pred_taken = 0;
  
  reg [31:0] ex_mem_rslt = 0, ex_mem_rrs2 = 0;
  reg [4:0]  ex_mem_rd = 0, ex_mem_op5 = 0;
  reg [4:0]  ex_mem_rs2 = 0; 
  reg        ex_mem_we = 0, ex_mem_mem_we = 0;
  
  reg [31:0] mem_wb_rslt = 0, mem_wb_ldd = 0;
  reg [4:0]  mem_wb_rd = 0, mem_wb_op5 = 0;
  reg        mem_wb_we = 0;

/********************************* IF *********************************/
  reg [31:0] r_pc = 0;
  wire [31:0] w_ir_if;
  m_amemory imem (w_clk, r_pc[13:2], 1'b0, 32'b0, w_ir_if);
  // (w_clk, w_addr, w_we, w_din, w_dout)

  // Branch Predictor
  wire [6:0] w_if_opcode = w_ir_if[6:0];
  wire w_if_is_branch = (w_if_opcode == 7'b1100011);

  wire [31:0] w_if_imm_b = {{20{w_ir_if[31]}}, w_ir_if[7], w_ir_if[30:25], w_ir_if[11:8], 1'b0};
  wire [31:0] w_if_pred_target = r_pc + w_if_imm_b;
  wire w_if_bht_val = bht[r_pc[7:2]][1];

  wire w_if_pred_taken = w_if_is_branch && w_if_bht_val;

/********************************* ID *********************************/
  wire [4:0]  w_op5_id = if_id_ir[6:2];
  wire [4:0]  w_rs1_id = if_id_ir[19:15];
  wire [4:0]  w_rs2_id = if_id_ir[24:20];
  wire [4:0]  w_rd_id  = if_id_ir[11:7];
  wire [31:0] w_imm_id, w_rdata1_id, w_rdata2_id;
  
  m_immgen immgen (if_id_ir, w_imm_id);
  
  wire [31:0] w_rslt2_wb; 
  m_regfile m_regs (w_clk, w_rs1_id, w_rs2_id, mem_wb_rd, w_ce & mem_wb_we, w_rslt2_wb, w_rdata1_id, w_rdata2_id);
  // (w_clk, w_rr1, w_rr2, w_wr, w_we, w_wdata, w_rdata1, w_rdata2)

  // Load Use Stall
  wire w_load_use_stall = (id_ex_op5 == 5'b00000) && (id_ex_rd != 0) && (id_ex_rd == w_rs1_id || id_ex_rd == w_rs2_id);

/********************************* EX *********************************/
  // Forwarding
  wire [31:0] w_fwd_alu_a = (ex_mem_we && ex_mem_rd != 0 && ex_mem_rd == id_ex_rs1) ? ex_mem_rslt :
                            (mem_wb_we && mem_wb_rd != 0 && mem_wb_rd == id_ex_rs1) ? w_rslt2_wb : id_ex_rrs1;

  wire [31:0] w_fwd_alu_b = (ex_mem_we && ex_mem_rd != 0 && ex_mem_rd == id_ex_rs2) ? ex_mem_rslt :
                            (mem_wb_we && mem_wb_rd != 0 && mem_wb_rd == id_ex_rs2) ? w_rslt2_wb : id_ex_rrs2;

  // jump
  wire w_branch_taken_ex = (id_ex_op5 == 5'b11000) && (id_ex_funct3[0] ^ (w_fwd_alu_a == w_fwd_alu_b));

  // ALU
  wire [31:0] w_ain_ex = (id_ex_op5 == 5'b01100) ? w_fwd_alu_b : id_ex_imm; 

  wire [31:0] w_add_ex = w_fwd_alu_a + w_ain_ex;
  wire [31:0] w_sll_ex = w_fwd_alu_a << w_ain_ex[4:0];
  wire [31:0] w_srl_ex = w_fwd_alu_a >> w_ain_ex[4:0];

  wire [31:0] w_rslt_ex = (id_ex_op5 != 5'b01100) ? w_add_ex:
                          (id_ex_funct3 == 3'b001) ? w_sll_ex:
                          (id_ex_funct3 == 3'b101) ? w_srl_ex:
                                                     w_add_ex;

  // Branch Predictor
  wire w_ex_is_branch = (id_ex_op5 == 5'b11000); 
  wire w_mispredict_ex = w_ex_is_branch && (w_branch_taken_ex != id_ex_pred_taken);
  wire [31:0] w_correct_pc_ex = w_branch_taken_ex ? (id_ex_pc + id_ex_imm) : (id_ex_pc + 4);

  always @(posedge w_clk) begin
    if (w_ce && w_ex_is_branch) begin
      case (bht[id_ex_pc[7:2]])
        2'b00: bht[id_ex_pc[7:2]] <= w_branch_taken_ex ? 2'b01 : 2'b00;
        2'b01: bht[id_ex_pc[7:2]] <= w_branch_taken_ex ? 2'b10 : 2'b00;
        2'b10: bht[id_ex_pc[7:2]] <= w_branch_taken_ex ? 2'b11 : 2'b01;
        2'b11: bht[id_ex_pc[7:2]] <= w_branch_taken_ex ? 2'b11 : 2'b10;
      endcase
    end
  end
/********************************* MEM *********************************/
  wire [31:0] w_ldd_mem;
  wire [31:0] w_mem_writedata = (mem_wb_we && mem_wb_rd != 0 && mem_wb_rd == ex_mem_rs2) ? w_rslt2_wb : ex_mem_rrs2;
  m_amemory dmem (w_clk, ex_mem_rslt[13:2], w_ce & ex_mem_mem_we, w_mem_writedata, w_ldd_mem);
  // (w_clk, w_addr, w_we, w_din, w_dout)

/********************************* WB *********************************/
  assign w_rslt2_wb = (mem_wb_op5 == 5'b00000) ? mem_wb_ldd : mem_wb_rslt;

  reg [31:0] r_led = 0;
  always @(posedge w_clk) if(w_ce & mem_wb_we & mem_wb_rd == 30) r_led <= w_rslt2_wb;
  assign w_led = r_led;

/********************************* clocking *********************************/
  always @(posedge w_clk) begin
    if (w_ce) begin
      // IF_PC
      r_pc <= w_mispredict_ex ? w_correct_pc_ex : 
              w_if_pred_taken ? w_if_pred_target : 
              w_load_use_stall ? r_pc : r_pc + 4;

      // IF/ID
      if (w_load_use_stall) begin
          if_id_pc <= if_id_pc;
          if_id_ir <= if_id_ir;
      end
      else if (w_mispredict_ex) begin
          if_id_ir <= {25'd0,7'b0010011}; // flush
          if_id_pc <= 0;
      end
      else begin
          if_id_pc <= r_pc;
          if_id_ir <= w_ir_if;
          if_id_pred_taken <= w_if_pred_taken;
      end

      // ID/EX
      if (w_load_use_stall || w_mispredict_ex) begin
          id_ex_we <= 0;
          id_ex_mem_we <= 0;
          id_ex_op5 <= 0; // avoid branching again
      end
      else begin
          id_ex_pc        <= if_id_pc;
          id_ex_rrs1      <= w_rdata1_id;
          id_ex_rrs2      <= w_rdata2_id;
          id_ex_rs1       <= w_rs1_id;
          id_ex_rs2       <= w_rs2_id;
          id_ex_imm       <= w_imm_id;
          id_ex_rd        <= w_rd_id;
          id_ex_op5       <= w_op5_id;
          id_ex_funct3    <= if_id_ir[14:12];
          id_ex_we        <= (w_op5_id==5'b01100 || w_op5_id==5'b00100 || w_op5_id==5'b00000);
          id_ex_mem_we    <= (w_op5_id==5'b01000);
          id_ex_pred_taken <= if_id_pred_taken;
      end

      // EX/MEM
      ex_mem_rslt     <= w_rslt_ex;
      ex_mem_rrs2     <= w_fwd_alu_b; 
      ex_mem_rs2      <= id_ex_rs2;   
      ex_mem_rd       <= id_ex_rd;
      ex_mem_we       <= id_ex_we;
      ex_mem_mem_we   <= id_ex_mem_we;
      ex_mem_op5      <= id_ex_op5;

      // MEM/WB
      mem_wb_rslt     <= ex_mem_rslt;
      mem_wb_ldd      <= w_ldd_mem;
      mem_wb_rd       <= ex_mem_rd;
      mem_wb_we       <= ex_mem_we;
      mem_wb_op5      <= ex_mem_op5;
    end
  end
endmodule

module m_regfile (w_clk, w_rr1, w_rr2, w_wr, w_we, w_wdata, w_rdata1, w_rdata2);
   input  wire        w_clk, w_we;
   input  wire [4:0]  w_rr1, w_rr2, w_wr;
   input  wire [31:0] w_wdata;
   output wire [31:0] w_rdata1, w_rdata2;
   reg [31:0] r[0:31];
   assign w_rdata1 = (w_rr1==0) ? 0 : (w_rr1==w_wr && w_we) ? w_wdata : r[w_rr1];
   assign w_rdata2 = (w_rr2==0) ? 0 : (w_rr2==w_wr && w_we) ? w_wdata : r[w_rr2];
   always @(posedge w_clk) if(w_we) r[w_wr] <= w_wdata;
endmodule

module m_amemory (w_clk, w_addr, w_we, w_din, w_dout);
  input  wire w_clk, w_we;
  input  wire [11:0] w_addr;
  input  wire [31:0] w_din;
  output wire [31:0] w_dout;
  reg [31:0] cm_ram [0:4095]; 
  always @(posedge w_clk) if (w_we) cm_ram[w_addr] <= w_din;
  assign w_dout = cm_ram[w_addr];
`include "program.txt"
endmodule

module m_immgen(w_i, r_imm); 
  input  wire [31:0] w_i;    
  output reg  [31:0] r_imm;  
  always @(*) case (w_i[6:2])
    5'b11000: r_imm <= {{20{w_i[31]}}, w_i[7], w_i[30:25], w_i[11:8], 1'b0};
    5'b01000: r_imm <= {{21{w_i[31]}}, w_i[30:25], w_i[11:7]};
    5'b11011: r_imm <= {{12{w_i[31]}}, w_i[19:12], w_i[20], w_i[30:21], 1'b0};
    5'b01101: r_imm <= {w_i[31:12], 12'b0};
    5'b00101: r_imm <= {w_i[31:12], 12'b0};
    default : r_imm <= {{21{w_i[31]}}, w_i[30:20]};
  endcase
endmodule