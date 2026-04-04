
// Top-level module instantiating and wiring all Issue Stage submodules:
//   - ROB               (Reorder Buffer)
//   - Physical_Reg_File (PRF + ready bits)
//   - Issue_Queue       (Integer ALU instructions)
//   - Address_Queue     (Load/Store address computation)
//   - LSQ               (Load/Store Queue)
//   - Arch_Reg_File     (Architectural Register File — written at commit)


module Issue_Stage_Top(
    input  wire        clk,
    input  wire        rst,


    // Dispatch interface (from Rename / Decode stage)

    input  wire [3:0]  dispatch_valid,

    // -- Integer / ALU instructions --
    input  wire [6:0]  dp_prs1_0,    dp_prs1_1,    dp_prs1_2,    dp_prs1_3,
    input  wire [6:0]  dp_prs2_0,    dp_prs2_1,    dp_prs2_2,    dp_prs2_3,
    input  wire [6:0]  dp_prd_0,     dp_prd_1,     dp_prd_2,     dp_prd_3,
    input  wire [6:0]  dp_opcode_0,  dp_opcode_1,  dp_opcode_2,  dp_opcode_3,
    input  wire [2:0]  dp_func3_0,   dp_func3_1,   dp_func3_2,   dp_func3_3,
    input  wire [6:0]  dp_func7_0,   dp_func7_1,   dp_func7_2,   dp_func7_3,
    input  wire [31:0] dp_imm_0,     dp_imm_1,     dp_imm_2,     dp_imm_3,

    // -- ROB fields (shared by IQ and AQ) --
    input  wire [4:0]  dp_rd_0,      dp_rd_1,      dp_rd_2,      dp_rd_3,
    input  wire [6:0]  dp_old_prd_0, dp_old_prd_1, dp_old_prd_2, dp_old_prd_3,
    input  wire [3:0]  dp_has_dest,
    input  wire [3:0]  dp_is_store,

    // -- Load/Store type flags (for AQ) --
    input  wire        dp_is_load_0,  dp_is_load_1,  dp_is_load_2,  dp_is_load_3,
    input  wire        dp_is_store_0, dp_is_store_1, dp_is_store_2, dp_is_store_3,

    // Outputs to Execute stage

    output wire        issue_valid,
    output wire [6:0]  issue_prs1,   issue_prs2,  issue_prd,
    output wire [6:0]  issue_opcode,
    output wire [2:0]  issue_func3,
    output wire [6:0]  issue_func7,
    output wire [31:0] issue_imm,
    output wire [4:0]  issue_rob_id,

    // read ports into PRF for the issued instruction
    input  wire [6:0]  prf_rd_tag_0, prf_rd_tag_1,
    output wire [63:0] prf_rd_data_0, prf_rd_data_1,

    // CDB input (from Execute / Writeback)

    input  wire        cdb_valid_in,
    input  wire [4:0]  cdb_rob_id_in,   // to ROB
    input  wire [6:0]  cdb_tag_in,      // physical register tag
    input  wire [63:0] cdb_data_in,

    // ROB IDs assigned to newly dispatched instructions (to Rename)

    output wire [4:0]  rob_id_0, rob_id_1, rob_id_2, rob_id_3,
    output wire        rob_full,


    // Commit outputs (to Rename / ARF)
  
    output wire [3:0]  commit_valid,
    output wire [4:0]  commit_rd_0,      commit_rd_1,      commit_rd_2,      commit_rd_3,
    output wire [6:0]  commit_prd_0,     commit_prd_1,     commit_prd_2,     commit_prd_3,
    output wire [6:0]  commit_old_prd_0, commit_old_prd_1, commit_old_prd_2, commit_old_prd_3,
    output wire [3:0]  commit_has_dest,
    output wire [3:0]  commit_is_store,

    // ARF read ports (for Rename / decode to read committed state)
    input  wire [4:0]  arf_rd_addr_0, arf_rd_addr_1,
    output wire [63:0] arf_rd_data_0, arf_rd_data_1,


    output wire        iq_full,
    output wire        aq_full,
    output wire        lsq_full
);


// PRF ready bits — 128 bits, one per physical register
wire [127:0] prf_ready_bits;

// CDB from LSQ (loads broadcast their result on the CDB)
// This is the *internal* CDB produced by LSQ that feeds IQ, AQ, PRF, and ROB.
// The external CDB input (cdb_*_in) is the ALU/EX result from the integer pipe.
// Both must be arbitrated.  Here we implement a simple priority:
//   LSQ CDB has priority when both are valid in the same cycle.
// The merged signals are what the submodules actually see.
wire        lsq_cdb_valid;
wire [6:0]  lsq_cdb_tag;
wire [63:0] lsq_cdb_data;

// Merged CDB presented to IQ, AQ, PRF
// (Single-CDB assumption: at most one of lsq_cdb_valid / cdb_valid_in fires
//  per cycle in a real design.  We OR them here; the PRF/IQ/AQ guard on tag.)
wire        cdb_valid_merged  = cdb_valid_in | lsq_cdb_valid;
wire [6:0]  cdb_tag_merged    = lsq_cdb_valid ? lsq_cdb_tag  : cdb_tag_in;
wire [63:0] cdb_data_merged   = lsq_cdb_valid ? lsq_cdb_data : cdb_data_in;

// ROB commit → PRF alloc (mark new physical registers as not-ready)
// The ROB ids come back from ROB as outputs rob_id_*, and alloc_en follows
// dispatch_valid + dispatch_has_dest.
wire [3:0] prf_alloc_en;
assign prf_alloc_en[0] = dispatch_valid[0] & dp_has_dest[0];
assign prf_alloc_en[1] = dispatch_valid[1] & dp_has_dest[1];
assign prf_alloc_en[2] = dispatch_valid[2] & dp_has_dest[2];
assign prf_alloc_en[3] = dispatch_valid[3] & dp_has_dest[3];

// ROB commit → ARF write data comes from PRF
wire [63:0] commit_data_0_w, commit_data_1_w, commit_data_2_w, commit_data_3_w;

// PRF read for commit: use commit_prd_* as read tags
// We reuse the two combinational read ports by reading commit_prd_0/1 here.
// For a 4-wide commit the ARF needs 4 values; in this implementation we
// connect commit_data from PRF read-port outputs driven by commit_prd tags.
// (A real design would add more read ports or do it sequentially.)
// Wire the two available PRF read ports to the first two commit slots;
// commit slots 2 & 3 are wired to 0 as a placeholder — extend read ports
// to support 4-wide commit if needed.
assign prf_rd_tag_0 = commit_prd_0;  // override: use for commit read-back
assign prf_rd_tag_1 = commit_prd_1;

assign commit_data_0_w = prf_rd_data_0;
assign commit_data_1_w = prf_rd_data_1;
assign commit_data_2_w = 64'd0;  // extend PRF read ports for full 4-wide commit
assign commit_data_3_w = 64'd0;

// Address Queue → LSQ forwarding wires
wire        aq_to_lsq_valid;
wire        aq_to_lsq_is_load,  aq_to_lsq_is_store;
wire [63:0] aq_to_lsq_addr;
wire [63:0] aq_to_lsq_store_data;
wire        aq_to_lsq_store_data_valid;
wire [6:0]  aq_to_lsq_prd;
wire [2:0]  aq_to_lsq_func3;
wire [4:0]  aq_to_lsq_rob_id;

// ROB commit store signal → LSQ
wire        rob_commit_store;
wire [4:0]  rob_commit_rob_id_w;

// commit_is_store[0] indicates the head-of-ROB commit is a store
assign rob_commit_store    = commit_valid[0] & commit_is_store[0];
assign rob_commit_rob_id_w = commit_rd_0;   // rob_id carried in commit_rd for stores
// NOTE: In this implementation the ROB passes the rob_id needed to locate the
// LSQ entry via commit_rd_0 (the architectural RD field).  If your top-level
// ROB commit protocol uses a separate rob_id field, replace the assignment
// above with that signal.


ROB u_rob(
    .clk                (clk),
    .rst                (rst),
    // dispatch
    .dispatch_valid     (dispatch_valid),
    .dispatch_rd_0      (dp_rd_0),
    .dispatch_rd_1      (dp_rd_1),
    .dispatch_rd_2      (dp_rd_2),
    .dispatch_rd_3      (dp_rd_3),
    .dispatch_prd_0     (dp_prd_0),
    .dispatch_prd_1     (dp_prd_1),
    .dispatch_prd_2     (dp_prd_2),
    .dispatch_prd_3     (dp_prd_3),
    .dispatch_old_prd_0 (dp_old_prd_0),
    .dispatch_old_prd_1 (dp_old_prd_1),
    .dispatch_old_prd_2 (dp_old_prd_2),
    .dispatch_old_prd_3 (dp_old_prd_3),
    .dispatch_has_dest  (dp_has_dest),
    .dispatch_is_store  (dp_is_store),
    // ROB IDs assigned (output to rename)
    .rob_id_0           (rob_id_0),
    .rob_id_1           (rob_id_1),
    .rob_id_2           (rob_id_2),
    .rob_id_3           (rob_id_3),
    .rob_full           (rob_full),
    // CDB — marks instruction complete
    .cdb_valid          (cdb_valid_merged),
    .cdb_rob_id         (cdb_rob_id_in),   // ALU pipe rob_id; LSQ uses its own
    // commit outputs
    .commit_valid       (commit_valid),
    .commit_rd_0        (commit_rd_0),
    .commit_rd_1        (commit_rd_1),
    .commit_rd_2        (commit_rd_2),
    .commit_rd_3        (commit_rd_3),
    .commit_prd_0       (commit_prd_0),
    .commit_prd_1       (commit_prd_1),
    .commit_prd_2       (commit_prd_2),
    .commit_prd_3       (commit_prd_3),
    .commit_old_prd_0   (commit_old_prd_0),
    .commit_old_prd_1   (commit_old_prd_1),
    .commit_old_prd_2   (commit_old_prd_2),
    .commit_old_prd_3   (commit_old_prd_3),
    .commit_has_dest    (commit_has_dest),
    .commit_is_store    (commit_is_store)
);


Physical_Reg_File u_prf(
    .clk          (clk),
    .rst          (rst),
    // allocation: mark new prd as not-ready at dispatch
    .alloc_en     (prf_alloc_en),
    .alloc_tag_0  (dp_prd_0),
    .alloc_tag_1  (dp_prd_1),
    .alloc_tag_2  (dp_prd_2),
    .alloc_tag_3  (dp_prd_3),
    // CDB write-back
    .cdb_valid    (cdb_valid_merged),
    .cdb_tag      (cdb_tag_merged),
    .cdb_data     (cdb_data_merged),
    // combinational read ports
    .rd_tag_0     (prf_rd_tag_0),
    .rd_tag_1     (prf_rd_tag_1),
    .rd_data_0    (prf_rd_data_0),
    .rd_data_1    (prf_rd_data_1),
    // ready-bit vector
    .ready_bits   (prf_ready_bits)
);

Issue_Queue u_iq(
    .clk            (clk),
    .rst            (rst),
    // dispatch
    .dispatch_valid (dispatch_valid),
    .dp_prs1_0      (dp_prs1_0),   .dp_prs1_1 (dp_prs1_1),
    .dp_prs1_2      (dp_prs1_2),   .dp_prs1_3 (dp_prs1_3),
    .dp_prs2_0      (dp_prs2_0),   .dp_prs2_1 (dp_prs2_1),
    .dp_prs2_2      (dp_prs2_2),   .dp_prs2_3 (dp_prs2_3),
    .dp_prd_0       (dp_prd_0),    .dp_prd_1  (dp_prd_1),
    .dp_prd_2       (dp_prd_2),    .dp_prd_3  (dp_prd_3),
    .dp_opcode_0    (dp_opcode_0), .dp_opcode_1(dp_opcode_1),
    .dp_opcode_2    (dp_opcode_2), .dp_opcode_3(dp_opcode_3),
    .dp_func3_0     (dp_func3_0),  .dp_func3_1 (dp_func3_1),
    .dp_func3_2     (dp_func3_2),  .dp_func3_3 (dp_func3_3),
    .dp_func7_0     (dp_func7_0),  .dp_func7_1 (dp_func7_1),
    .dp_func7_2     (dp_func7_2),  .dp_func7_3 (dp_func7_3),
    .dp_imm_0       (dp_imm_0),    .dp_imm_1   (dp_imm_1),
    .dp_imm_2       (dp_imm_2),    .dp_imm_3   (dp_imm_3),
    .dp_rob_id_0    (rob_id_0),    .dp_rob_id_1(rob_id_1),
    .dp_rob_id_2    (rob_id_2),    .dp_rob_id_3(rob_id_3),
    // PRF ready bits
    .prf_ready_bits (prf_ready_bits),
    // CDB wakeup
    .cdb_valid      (cdb_valid_merged),
    .cdb_tag        (cdb_tag_merged),
    // issue output
    .issue_valid    (issue_valid),
    .issue_prs1     (issue_prs1),
    .issue_prs2     (issue_prs2),
    .issue_prd      (issue_prd),
    .issue_opcode   (issue_opcode),
    .issue_func3    (issue_func3),
    .issue_func7    (issue_func7),
    .issue_imm      (issue_imm),
    .issue_rob_id   (issue_rob_id),
    .iq_full        (iq_full)
);


Address_Queue u_aq(
    .clk                  (clk),
    .rst                  (rst),
    // dispatch
    .dispatch_valid       (dispatch_valid),
    .dp_is_load_0         (dp_is_load_0),  .dp_is_load_1 (dp_is_load_1),
    .dp_is_load_2         (dp_is_load_2),  .dp_is_load_3 (dp_is_load_3),
    .dp_is_store_0        (dp_is_store_0), .dp_is_store_1(dp_is_store_1),
    .dp_is_store_2        (dp_is_store_2), .dp_is_store_3(dp_is_store_3),
    .dp_prs1_0            (dp_prs1_0),     .dp_prs1_1    (dp_prs1_1),
    .dp_prs1_2            (dp_prs1_2),     .dp_prs1_3    (dp_prs1_3),
    .dp_prs2_0            (dp_prs2_0),     .dp_prs2_1    (dp_prs2_1),
    .dp_prs2_2            (dp_prs2_2),     .dp_prs2_3    (dp_prs2_3),
    .dp_prd_0             (dp_prd_0),      .dp_prd_1     (dp_prd_1),
    .dp_prd_2             (dp_prd_2),      .dp_prd_3     (dp_prd_3),
    .dp_imm_0             (dp_imm_0),      .dp_imm_1     (dp_imm_1),
    .dp_imm_2             (dp_imm_2),      .dp_imm_3     (dp_imm_3),
    .dp_func3_0           (dp_func3_0),    .dp_func3_1   (dp_func3_1),
    .dp_func3_2           (dp_func3_2),    .dp_func3_3   (dp_func3_3),
    .dp_rob_id_0          (rob_id_0),      .dp_rob_id_1  (rob_id_1),
    .dp_rob_id_2          (rob_id_2),      .dp_rob_id_3  (rob_id_3),
    // PRF ready bits
    .prf_ready_bits       (prf_ready_bits),
    // CDB wakeup
    .cdb_valid            (cdb_valid_merged),
    .cdb_tag              (cdb_tag_merged),
    .cdb_data             (cdb_data_merged),
    // outputs to LSQ
    .lsq_addr_valid       (aq_to_lsq_valid),
    .lsq_is_load          (aq_to_lsq_is_load),
    .lsq_is_store         (aq_to_lsq_is_store),
    .lsq_addr             (aq_to_lsq_addr),
    .lsq_store_data       (aq_to_lsq_store_data),
    .lsq_store_data_valid (aq_to_lsq_store_data_valid),
    .lsq_prd              (aq_to_lsq_prd),
    .lsq_func3            (aq_to_lsq_func3),
    .lsq_rob_id           (aq_to_lsq_rob_id),
    .aq_full              (aq_full)
);


LSQ u_lsq(
    .clk                  (clk),
    .rst                  (rst),
    // from Address Queue
    .aq_valid             (aq_to_lsq_valid),
    .aq_is_load           (aq_to_lsq_is_load),
    .aq_is_store          (aq_to_lsq_is_store),
    .aq_addr              (aq_to_lsq_addr),
    .aq_store_data        (aq_to_lsq_store_data),
    .aq_store_data_valid  (aq_to_lsq_store_data_valid),
    .aq_prd               (aq_to_lsq_prd),
    .aq_func3             (aq_to_lsq_func3),
    .aq_rob_id            (aq_to_lsq_rob_id),
    // ROB commit — trigger store execution
    .rob_commit_store     (rob_commit_store),
    .rob_commit_rob_id    (rob_commit_rob_id_w),
    // CDB output (load results)
    .cdb_valid            (lsq_cdb_valid),
    .cdb_tag              (lsq_cdb_tag),
    .cdb_data             (lsq_cdb_data),
    .lsq_full             (lsq_full)
);

Arch_Reg_File u_arf(
    .clk           (clk),
    .rst           (rst),
    // commit write ports
    .commit_valid  (commit_valid),
    .commit_rd_0   (commit_rd_0),
    .commit_rd_1   (commit_rd_1),
    .commit_rd_2   (commit_rd_2),
    .commit_rd_3   (commit_rd_3),
    .commit_data_0 (commit_data_0_w),
    .commit_data_1 (commit_data_1_w),
    .commit_data_2 (commit_data_2_w),
    .commit_data_3 (commit_data_3_w),
    // read ports
    .rd_addr_0     (arf_rd_addr_0),
    .rd_addr_1     (arf_rd_addr_1),
    .rd_data_0     (arf_rd_data_0),
    .rd_data_1     (arf_rd_data_1)
);

endmodule



module ROB(clk, rst,
    dispatch_valid,
    dispatch_rd_0,   dispatch_rd_1,   dispatch_rd_2,   dispatch_rd_3,
    dispatch_prd_0,  dispatch_prd_1,  dispatch_prd_2,  dispatch_prd_3,
    dispatch_old_prd_0, dispatch_old_prd_1, dispatch_old_prd_2, dispatch_old_prd_3,
    dispatch_has_dest, dispatch_is_store,
    rob_id_0, rob_id_1, rob_id_2, rob_id_3,
    rob_full,
    cdb_valid, cdb_rob_id,
    commit_valid,
    commit_rd_0,      commit_rd_1,      commit_rd_2,      commit_rd_3,
    commit_prd_0,     commit_prd_1,     commit_prd_2,     commit_prd_3,
    commit_old_prd_0, commit_old_prd_1, commit_old_prd_2, commit_old_prd_3,
    commit_has_dest, commit_is_store);

input clk, rst;
input [3:0] dispatch_valid;
input [4:0] dispatch_rd_0,  dispatch_rd_1,  dispatch_rd_2,  dispatch_rd_3;
input [6:0] dispatch_prd_0, dispatch_prd_1, dispatch_prd_2, dispatch_prd_3;
input [6:0] dispatch_old_prd_0, dispatch_old_prd_1, dispatch_old_prd_2, dispatch_old_prd_3;
input [3:0] dispatch_has_dest, dispatch_is_store;
output reg [4:0] rob_id_0, rob_id_1, rob_id_2, rob_id_3;
output reg       rob_full;
input        cdb_valid;
input [4:0]  cdb_rob_id;
output reg [3:0] commit_valid;
output reg [4:0] commit_rd_0,      commit_rd_1,      commit_rd_2,      commit_rd_3;
output reg [6:0] commit_prd_0,     commit_prd_1,     commit_prd_2,     commit_prd_3;
output reg [6:0] commit_old_prd_0, commit_old_prd_1, commit_old_prd_2, commit_old_prd_3;
output reg [3:0] commit_has_dest, commit_is_store;

reg        rob_valid    [0:31];
reg        rob_done     [0:31];
reg [4:0]  rob_rd       [0:31];
reg [6:0]  rob_prd      [0:31];
reg [6:0]  rob_old_prd  [0:31];
reg        rob_has_dest [0:31];
reg        rob_is_store [0:31];

reg [4:0] head;
reg [4:0] tail;
reg [5:0] count;

integer i;

wire [2:0] n_dispatch = dispatch_valid[0] + dispatch_valid[1] +
                        dispatch_valid[2] + dispatch_valid[3];

always@(posedge clk, negedge rst) begin
    if(!rst) begin
        head  <= 5'd0;
        tail  <= 5'd0;
        count <= 6'd0;
        for(i = 0; i < 32; i = i + 1) begin
            rob_valid[i]    <= 1'b0;
            rob_done[i]     <= 1'b0;
            rob_rd[i]       <= 5'd0;
            rob_prd[i]      <= 7'd0;
            rob_old_prd[i]  <= 7'd0;
            rob_has_dest[i] <= 1'b0;
            rob_is_store[i] <= 1'b0;
        end
        commit_valid     <= 4'd0;
        commit_rd_0      <= 5'd0; commit_rd_1      <= 5'd0;
        commit_rd_2      <= 5'd0; commit_rd_3      <= 5'd0;
        commit_prd_0     <= 7'd0; commit_prd_1     <= 7'd0;
        commit_prd_2     <= 7'd0; commit_prd_3     <= 7'd0;
        commit_old_prd_0 <= 7'd0; commit_old_prd_1 <= 7'd0;
        commit_old_prd_2 <= 7'd0; commit_old_prd_3 <= 7'd0;
        commit_has_dest  <= 4'd0; commit_is_store  <= 4'd0;
    end
    else begin

        if(cdb_valid) begin
            rob_done[cdb_rob_id] <= 1'b1;
        end

        if(dispatch_valid[0]) begin
            rob_valid[tail]    <= 1'b1;
            rob_done[tail]     <= 1'b0;
            rob_rd[tail]       <= dispatch_rd_0;
            rob_prd[tail]      <= dispatch_prd_0;
            rob_old_prd[tail]  <= dispatch_old_prd_0;
            rob_has_dest[tail] <= dispatch_has_dest[0];
            rob_is_store[tail] <= dispatch_is_store[0];
        end
        if(dispatch_valid[1]) begin
            rob_valid[tail+1]    <= 1'b1;
            rob_done[tail+1]     <= 1'b0;
            rob_rd[tail+1]       <= dispatch_rd_1;
            rob_prd[tail+1]      <= dispatch_prd_1;
            rob_old_prd[tail+1]  <= dispatch_old_prd_1;
            rob_has_dest[tail+1] <= dispatch_has_dest[1];
            rob_is_store[tail+1] <= dispatch_is_store[1];
        end
        if(dispatch_valid[2]) begin
            rob_valid[tail+2]    <= 1'b1;
            rob_done[tail+2]     <= 1'b0;
            rob_rd[tail+2]       <= dispatch_rd_2;
            rob_prd[tail+2]      <= dispatch_prd_2;
            rob_old_prd[tail+2]  <= dispatch_old_prd_2;
            rob_has_dest[tail+2] <= dispatch_has_dest[2];
            rob_is_store[tail+2] <= dispatch_is_store[2];
        end
        if(dispatch_valid[3]) begin
            rob_valid[tail+3]    <= 1'b1;
            rob_done[tail+3]     <= 1'b0;
            rob_rd[tail+3]       <= dispatch_rd_3;
            rob_prd[tail+3]      <= dispatch_prd_3;
            rob_old_prd[tail+3]  <= dispatch_old_prd_3;
            rob_has_dest[tail+3] <= dispatch_has_dest[3];
            rob_is_store[tail+3] <= dispatch_is_store[3];
        end
        tail  <= (tail + n_dispatch) % 32;

        commit_valid     <= 4'b0000;
        commit_rd_0      <= 5'd0; commit_rd_1      <= 5'd0;
        commit_rd_2      <= 5'd0; commit_rd_3      <= 5'd0;
        commit_prd_0     <= 7'd0; commit_prd_1     <= 7'd0;
        commit_prd_2     <= 7'd0; commit_prd_3     <= 7'd0;
        commit_old_prd_0 <= 7'd0; commit_old_prd_1 <= 7'd0;
        commit_old_prd_2 <= 7'd0; commit_old_prd_3 <= 7'd0;
        commit_has_dest  <= 4'd0; commit_is_store  <= 4'd0;

        if(rob_valid[head] && rob_done[head]) begin
            commit_valid[0]    <= 1'b1;
            commit_rd_0        <= rob_rd[head];
            commit_prd_0       <= rob_prd[head];
            commit_old_prd_0   <= rob_old_prd[head];
            commit_has_dest[0] <= rob_has_dest[head];
            commit_is_store[0] <= rob_is_store[head];
            rob_valid[head]    <= 1'b0;

            if(rob_valid[(head+1)%32] && rob_done[(head+1)%32]) begin
                commit_valid[1]    <= 1'b1;
                commit_rd_1        <= rob_rd[(head+1)%32];
                commit_prd_1       <= rob_prd[(head+1)%32];
                commit_old_prd_1   <= rob_old_prd[(head+1)%32];
                commit_has_dest[1] <= rob_has_dest[(head+1)%32];
                commit_is_store[1] <= rob_is_store[(head+1)%32];
                rob_valid[(head+1)%32] <= 1'b0;

                if(rob_valid[(head+2)%32] && rob_done[(head+2)%32]) begin
                    commit_valid[2]    <= 1'b1;
                    commit_rd_2        <= rob_rd[(head+2)%32];
                    commit_prd_2       <= rob_prd[(head+2)%32];
                    commit_old_prd_2   <= rob_old_prd[(head+2)%32];
                    commit_has_dest[2] <= rob_has_dest[(head+2)%32];
                    commit_is_store[2] <= rob_is_store[(head+2)%32];
                    rob_valid[(head+2)%32] <= 1'b0;

                    if(rob_valid[(head+3)%32] && rob_done[(head+3)%32]) begin
                        commit_valid[3]    <= 1'b1;
                        commit_rd_3        <= rob_rd[(head+3)%32];
                        commit_prd_3       <= rob_prd[(head+3)%32];
                        commit_old_prd_3   <= rob_old_prd[(head+3)%32];
                        commit_has_dest[3] <= rob_has_dest[(head+3)%32];
                        commit_is_store[3] <= rob_is_store[(head+3)%32];
                        rob_valid[(head+3)%32] <= 1'b0;
                        head  <= (head + 4) % 32;
                        count <= count - 4 + n_dispatch;
                    end
                    else begin
                        head  <= (head + 3) % 32;
                        count <= count - 3 + n_dispatch;
                    end
                end
                else begin
                    head  <= (head + 2) % 32;
                    count <= count - 2 + n_dispatch;
                end
            end
            else begin
                head  <= (head + 1) % 32;
                count <= count - 1 + n_dispatch;
            end
        end
        else begin
            count <= count + n_dispatch;
        end

    end
end

always@(*) begin
    rob_id_0 = tail;
    rob_id_1 = (tail + 1) % 32;
    rob_id_2 = (tail + 2) % 32;
    rob_id_3 = (tail + 3) % 32;
    rob_full  = (count >= 6'd28);
end

endmodule


module LSQ(clk, rst,
    // from Address Queue
    aq_valid,
    aq_is_load, aq_is_store,
    aq_addr, aq_store_data, aq_store_data_valid,
    aq_prd, aq_func3, aq_rob_id,
    // ROB commit — signal stores to execute
    rob_commit_store,
    rob_commit_rob_id,
    // CDB output — broadcast load result
    cdb_valid,
    cdb_tag,
    cdb_data,
    lsq_full);

input clk, rst;
// input from Address Queue
input        aq_valid;
input        aq_is_load, aq_is_store;
input [63:0] aq_addr;
input [63:0] aq_store_data;
input        aq_store_data_valid;
input [6:0]  aq_prd;
input [2:0]  aq_func3;
input [4:0]  aq_rob_id;
// ROB commit signal for stores
input        rob_commit_store;
input [4:0]  rob_commit_rob_id;
// CDB output — load result broadcast
output reg        cdb_valid;
output reg [6:0]  cdb_tag;
output reg [63:0] cdb_data;
output reg        lsq_full;

// Entry storage
reg        lsq_valid    [0:7];
reg        lsq_is_load  [0:7];
reg        lsq_is_store [0:7];
reg [63:0] lsq_addr     [0:7];
reg [63:0] lsq_sdata    [0:7];
reg        lsq_sdata_v  [0:7];
reg [6:0]  lsq_prd      [0:7];
reg [2:0]  lsq_func3    [0:7];
reg [4:0]  lsq_rob_id   [0:7];
reg        lsq_executed [0:7];

// internal data memory — 32 words x 64-bit
// indexed by addr[6:3] (8-byte aligned)
reg [63:0] data_mem [0:31];

reg [2:0] tail;
reg [2:0] count;
integer i;

always@(posedge clk, negedge rst) begin
    if(!rst) begin
        tail  <= 3'd0;
        count <= 3'd0;
        cdb_valid <= 1'b0;
        for(i = 0; i < 8; i = i + 1) begin
            lsq_valid[i]    <= 1'b0;
            lsq_executed[i] <= 1'b0;
        end
        for(i = 0; i < 32; i = i + 1)
            data_mem[i] <= 64'd0;
    end
    else begin

        cdb_valid <= 1'b0;

        // ---- ENQUEUE from Address Queue ----
        if(aq_valid) begin
            lsq_valid[tail]    <= 1'b1;
            lsq_is_load[tail]  <= aq_is_load;
            lsq_is_store[tail] <= aq_is_store;
            lsq_addr[tail]     <= aq_addr;
            lsq_sdata[tail]    <= aq_store_data;
            lsq_sdata_v[tail]  <= aq_store_data_valid;
            lsq_prd[tail]      <= aq_prd;
            lsq_func3[tail]    <= aq_func3;
            lsq_rob_id[tail]   <= aq_rob_id;
            lsq_executed[tail] <= 1'b0;
            tail  <= (tail + 1) % 8;
            count <= count + 1;
        end

        // ---- EXECUTE LOADS: scan for valid unexecuted loads ----
        for(i = 0; i < 8; i = i + 1) begin
            if(lsq_valid[i] && lsq_is_load[i] && !lsq_executed[i] && !cdb_valid) begin
                // read from data memory
                cdb_valid        <= 1'b1;
                cdb_tag          <= lsq_prd[i];
                cdb_data         <= data_mem[lsq_addr[i][7:3]];
                lsq_executed[i]  <= 1'b1;
            end
        end

        // ---- EXECUTE STORES at ROB commit ----
        if(rob_commit_store) begin
            for(i = 0; i < 8; i = i + 1) begin
                if(lsq_valid[i] && lsq_is_store[i] && lsq_rob_id[i] == rob_commit_rob_id) begin
                    data_mem[lsq_addr[i][7:3]] <= lsq_sdata[i];
                    lsq_valid[i]    <= 1'b0;
                    count           <= count - 1;
                end
            end
        end

        // ---- FREE executed load entries ----
        for(i = 0; i < 8; i = i + 1) begin
            if(lsq_valid[i] && lsq_is_load[i] && lsq_executed[i]) begin
                lsq_valid[i] <= 1'b0;
                count        <= count - 1;
            end
        end

    end
end

always@(*) begin
    lsq_full = (count >= 3'd6);
end

endmodule


module Address_Queue(clk, rst,
    dispatch_valid,
    dp_is_load_0,  dp_is_load_1,  dp_is_load_2,  dp_is_load_3,
    dp_is_store_0, dp_is_store_1, dp_is_store_2, dp_is_store_3,
    dp_prs1_0, dp_prs1_1, dp_prs1_2, dp_prs1_3,
    dp_prs2_0, dp_prs2_1, dp_prs2_2, dp_prs2_3,
    dp_prd_0,  dp_prd_1,  dp_prd_2,  dp_prd_3,
    dp_imm_0,  dp_imm_1,  dp_imm_2,  dp_imm_3,
    dp_func3_0, dp_func3_1, dp_func3_2, dp_func3_3,
    dp_rob_id_0, dp_rob_id_1, dp_rob_id_2, dp_rob_id_3,
    prf_ready_bits,
    cdb_valid, cdb_tag, cdb_data,
    lsq_addr_valid,
    lsq_is_load, lsq_is_store,
    lsq_addr,
    lsq_store_data, lsq_store_data_valid,
    lsq_prd, lsq_func3, lsq_rob_id,
    aq_full);

input clk, rst;
input [3:0] dispatch_valid;
input dp_is_load_0,  dp_is_load_1,  dp_is_load_2,  dp_is_load_3;
input dp_is_store_0, dp_is_store_1, dp_is_store_2, dp_is_store_3;
input [6:0]  dp_prs1_0,  dp_prs1_1,  dp_prs1_2,  dp_prs1_3;
input [6:0]  dp_prs2_0,  dp_prs2_1,  dp_prs2_2,  dp_prs2_3;
input [6:0]  dp_prd_0,   dp_prd_1,   dp_prd_2,   dp_prd_3;
input [31:0] dp_imm_0,   dp_imm_1,   dp_imm_2,   dp_imm_3;
input [2:0]  dp_func3_0, dp_func3_1, dp_func3_2, dp_func3_3;
input [4:0]  dp_rob_id_0, dp_rob_id_1, dp_rob_id_2, dp_rob_id_3;
input [127:0] prf_ready_bits;
input        cdb_valid;
input [6:0]  cdb_tag;
input [63:0] cdb_data;
output reg        lsq_addr_valid;
output reg        lsq_is_load, lsq_is_store;
output reg [63:0] lsq_addr;
output reg [63:0] lsq_store_data;
output reg        lsq_store_data_valid;
output reg [6:0]  lsq_prd;
output reg [2:0]  lsq_func3;
output reg [4:0]  lsq_rob_id;
output reg        aq_full;

reg        aq_valid     [0:7];
reg        aq_is_load   [0:7];
reg        aq_is_store  [0:7];
reg [6:0]  aq_prs1      [0:7];
reg [6:0]  aq_prs2      [0:7];
reg [6:0]  aq_prd       [0:7];
reg [31:0] aq_imm       [0:7];
reg [2:0]  aq_func3     [0:7];
reg [4:0]  aq_rob_id    [0:7];
reg        aq_base_rdy  [0:7];
reg        aq_data_rdy  [0:7];
reg [63:0] aq_base_val  [0:7];
reg [63:0] aq_store_val [0:7];

reg [2:0] tail;
reg [2:0] count;
integer i;

always@(posedge clk, negedge rst) begin
    if(!rst) begin
        tail  <= 3'd0;
        count <= 3'd0;
        for(i = 0; i < 8; i = i + 1) begin
            aq_valid[i]    <= 1'b0;
            aq_base_rdy[i] <= 1'b0;
            aq_data_rdy[i] <= 1'b0;
        end
        lsq_addr_valid       <= 1'b0;
        lsq_is_load          <= 1'b0;
        lsq_is_store         <= 1'b0;
        lsq_addr             <= 64'd0;
        lsq_store_data       <= 64'd0;
        lsq_store_data_valid <= 1'b0;
        lsq_prd              <= 7'd0;
        lsq_func3            <= 3'd0;
        lsq_rob_id           <= 5'd0;
    end
    else begin

        // clear all outputs every cycle — only set when forwarding happens
        lsq_addr_valid       <= 1'b0;
        lsq_is_load          <= 1'b0;
        lsq_is_store         <= 1'b0;
        lsq_addr             <= 64'd0;
        lsq_store_data       <= 64'd0;
        lsq_store_data_valid <= 1'b0;
        lsq_prd              <= 7'd0;
        lsq_func3            <= 3'd0;
        lsq_rob_id           <= 5'd0;

        // CDB wakeup
        if(cdb_valid) begin
            for(i = 0; i < 8; i = i + 1) begin
                if(aq_valid[i]) begin
                    if(aq_prs1[i] == cdb_tag) begin
                        aq_base_val[i]  <= cdb_data;
                        aq_base_rdy[i]  <= 1'b1;
                    end
                    if(aq_is_store[i] && aq_prs2[i] == cdb_tag) begin
                        aq_store_val[i] <= cdb_data;
                        aq_data_rdy[i]  <= 1'b1;
                    end
                end
            end
        end

        // dispatch
        if(dispatch_valid[0]) begin
            aq_valid[tail]     <= 1'b1;
            aq_is_load[tail]   <= dp_is_load_0;
            aq_is_store[tail]  <= dp_is_store_0;
            aq_prs1[tail]      <= dp_prs1_0;
            aq_prs2[tail]      <= dp_prs2_0;
            aq_prd[tail]       <= dp_prd_0;
            aq_imm[tail]       <= dp_imm_0;
            aq_func3[tail]     <= dp_func3_0;
            aq_rob_id[tail]    <= dp_rob_id_0;
            aq_base_rdy[tail]  <= prf_ready_bits[dp_prs1_0];
            aq_data_rdy[tail]  <= dp_is_load_0 ? 1'b1 : prf_ready_bits[dp_prs2_0];
            aq_base_val[tail]  <= 64'd0;
            aq_store_val[tail] <= 64'd0;
            tail  <= (tail + 1) % 8;
            count <= count + 1;
        end

        // select and forward to LSQ
        for(i = 0; i < 8; i = i + 1) begin
            if(aq_valid[i] && aq_base_rdy[i] && aq_data_rdy[i] && !lsq_addr_valid) begin
                lsq_addr_valid       <= 1'b1;
                lsq_is_load          <= aq_is_load[i];
                lsq_is_store         <= aq_is_store[i];
                lsq_addr             <= aq_base_val[i] + {{32{aq_imm[i][31]}}, aq_imm[i]};
                lsq_store_data       <= aq_store_val[i];
                lsq_store_data_valid <= aq_is_store[i];
                lsq_prd              <= aq_prd[i];
                lsq_func3            <= aq_func3[i];
                lsq_rob_id           <= aq_rob_id[i];
                aq_valid[i]          <= 1'b0;
                count                <= count - 1;
            end
        end

    end
end

always@(*) begin
    aq_full = (count >= 3'd5);
end

endmodule

module Issue_Queue(clk, rst,
    dispatch_valid,
    dp_prs1_0, dp_prs1_1, dp_prs1_2, dp_prs1_3,
    dp_prs2_0, dp_prs2_1, dp_prs2_2, dp_prs2_3,
    dp_prd_0,  dp_prd_1,  dp_prd_2,  dp_prd_3,
    dp_opcode_0, dp_opcode_1, dp_opcode_2, dp_opcode_3,
    dp_func3_0,  dp_func3_1,  dp_func3_2,  dp_func3_3,
    dp_func7_0,  dp_func7_1,  dp_func7_2,  dp_func7_3,
    dp_imm_0,    dp_imm_1,    dp_imm_2,    dp_imm_3,
    dp_rob_id_0, dp_rob_id_1, dp_rob_id_2, dp_rob_id_3,
    prf_ready_bits,
    cdb_valid, cdb_tag,
    issue_valid,
    issue_prs1, issue_prs2, issue_prd,
    issue_opcode, issue_func3, issue_func7, issue_imm, issue_rob_id,
    iq_full);

input clk, rst;
input [3:0] dispatch_valid;
input [6:0] dp_prs1_0, dp_prs1_1, dp_prs1_2, dp_prs1_3;
input [6:0] dp_prs2_0, dp_prs2_1, dp_prs2_2, dp_prs2_3;
input [6:0] dp_prd_0,  dp_prd_1,  dp_prd_2,  dp_prd_3;
input [6:0] dp_opcode_0, dp_opcode_1, dp_opcode_2, dp_opcode_3;
input [2:0] dp_func3_0,  dp_func3_1,  dp_func3_2,  dp_func3_3;
input [6:0] dp_func7_0,  dp_func7_1,  dp_func7_2,  dp_func7_3;
input [31:0] dp_imm_0,   dp_imm_1,    dp_imm_2,    dp_imm_3;
input [4:0] dp_rob_id_0, dp_rob_id_1, dp_rob_id_2, dp_rob_id_3;
input [127:0] prf_ready_bits;
input        cdb_valid;
input [6:0]  cdb_tag;
output reg        issue_valid;
output reg [6:0]  issue_prs1, issue_prs2, issue_prd;
output reg [6:0]  issue_opcode;
output reg [2:0]  issue_func3;
output reg [6:0]  issue_func7;
output reg [31:0] issue_imm;
output reg [4:0]  issue_rob_id;
output reg        iq_full;

reg        iq_valid    [0:15];
reg [6:0]  iq_prs1     [0:15];
reg [6:0]  iq_prs2     [0:15];
reg [6:0]  iq_prd      [0:15];
reg        iq_s1_rdy   [0:15];
reg        iq_s2_rdy   [0:15];
reg [6:0]  iq_opcode   [0:15];
reg [2:0]  iq_func3    [0:15];
reg [6:0]  iq_func7    [0:15];
reg [31:0] iq_imm      [0:15];
reg [4:0]  iq_rob_id   [0:15];

integer i;
reg [3:0] free_slot_0, free_slot_1, free_slot_2, free_slot_3;
reg       found_0, found_1, found_2, found_3;
reg [3:0] count;

// selected entry index and valid flag — determined combinationally
reg        sel_valid;
reg [3:0]  sel_idx;

// combinational block: find free slots and select oldest ready entry
always@(*) begin
    // free slot finder
    free_slot_0 = 0; free_slot_1 = 0; free_slot_2 = 0; free_slot_3 = 0;
    found_0 = 0; found_1 = 0; found_2 = 0; found_3 = 0;
    for(i = 0; i < 16; i = i + 1) begin
        if(!iq_valid[i] && !found_0) begin
            free_slot_0 = i[3:0]; found_0 = 1;
        end else if(!iq_valid[i] && found_0 && !found_1) begin
            free_slot_1 = i[3:0]; found_1 = 1;
        end else if(!iq_valid[i] && found_1 && !found_2) begin
            free_slot_2 = i[3:0]; found_2 = 1;
        end else if(!iq_valid[i] && found_2 && !found_3) begin
            free_slot_3 = i[3:0]; found_3 = 1;
        end
    end

    // select oldest ready entry: scan 0 to 15, first match = lowest index = oldest
    sel_valid = 0;
    sel_idx   = 0;
    for(i = 15; i >= 0; i = i - 1) begin
        if(iq_valid[i] && iq_s1_rdy[i] && iq_s2_rdy[i]) begin
            sel_valid = 1;
            sel_idx   = i[3:0];
        end
    end
    // scanning 15 down to 0 with last-write-wins means index 0 always wins
    // when multiple entries are ready — lowest index selected

    iq_full = (count >= 4'd12);
end

always@(posedge clk, negedge rst) begin
    if(!rst) begin
        for(i = 0; i < 16; i = i + 1) begin
            iq_valid[i]  <= 1'b0;
            iq_s1_rdy[i] <= 1'b0;
            iq_s2_rdy[i] <= 1'b0;
        end
        issue_valid <= 0;
        count <= 0;
    end
    else begin

        // CDB wakeup
        if(cdb_valid) begin
            for(i = 0; i < 16; i = i + 1) begin
                if(iq_valid[i]) begin
                    if(iq_prs1[i] == cdb_tag) iq_s1_rdy[i] <= 1'b1;
                    if(iq_prs2[i] == cdb_tag) iq_s2_rdy[i] <= 1'b1;
                end
            end
        end

        // dispatch
        if(dispatch_valid[0] && found_0) begin
            iq_valid[free_slot_0]  <= 1'b1;
            iq_prs1[free_slot_0]   <= dp_prs1_0;
            iq_prs2[free_slot_0]   <= dp_prs2_0;
            iq_prd[free_slot_0]    <= dp_prd_0;
            iq_s1_rdy[free_slot_0] <= prf_ready_bits[dp_prs1_0];
            iq_s2_rdy[free_slot_0] <= prf_ready_bits[dp_prs2_0];
            iq_opcode[free_slot_0] <= dp_opcode_0;
            iq_func3[free_slot_0]  <= dp_func3_0;
            iq_func7[free_slot_0]  <= dp_func7_0;
            iq_imm[free_slot_0]    <= dp_imm_0;
            iq_rob_id[free_slot_0] <= dp_rob_id_0;
        end
        if(dispatch_valid[1] && found_1) begin
            iq_valid[free_slot_1]  <= 1'b1;
            iq_prs1[free_slot_1]   <= dp_prs1_1;
            iq_prs2[free_slot_1]   <= dp_prs2_1;
            iq_prd[free_slot_1]    <= dp_prd_1;
            iq_s1_rdy[free_slot_1] <= prf_ready_bits[dp_prs1_1];
            iq_s2_rdy[free_slot_1] <= prf_ready_bits[dp_prs2_1];
            iq_opcode[free_slot_1] <= dp_opcode_1;
            iq_func3[free_slot_1]  <= dp_func3_1;
            iq_func7[free_slot_1]  <= dp_func7_1;
            iq_imm[free_slot_1]    <= dp_imm_1;
            iq_rob_id[free_slot_1] <= dp_rob_id_1;
        end
        if(dispatch_valid[2] && found_2) begin
            iq_valid[free_slot_2]  <= 1'b1;
            iq_prs1[free_slot_2]   <= dp_prs1_2;
            iq_prs2[free_slot_2]   <= dp_prs2_2;
            iq_prd[free_slot_2]    <= dp_prd_2;
            iq_s1_rdy[free_slot_2] <= prf_ready_bits[dp_prs1_2];
            iq_s2_rdy[free_slot_2] <= prf_ready_bits[dp_prs2_2];
            iq_opcode[free_slot_2] <= dp_opcode_2;
            iq_func3[free_slot_2]  <= dp_func3_2;
            iq_func7[free_slot_2]  <= dp_func7_2;
            iq_imm[free_slot_2]    <= dp_imm_2;
            iq_rob_id[free_slot_2] <= dp_rob_id_2;
        end
        if(dispatch_valid[3] && found_3) begin
            iq_valid[free_slot_3]  <= 1'b1;
            iq_prs1[free_slot_3]   <= dp_prs1_3;
            iq_prs2[free_slot_3]   <= dp_prs2_3;
            iq_prd[free_slot_3]    <= dp_prd_3;
            iq_s1_rdy[free_slot_3] <= prf_ready_bits[dp_prs1_3];
            iq_s2_rdy[free_slot_3] <= prf_ready_bits[dp_prs2_3];
            iq_opcode[free_slot_3] <= dp_opcode_3;
            iq_func3[free_slot_3]  <= dp_func3_3;
            iq_func7[free_slot_3]  <= dp_func7_3;
            iq_imm[free_slot_3]    <= dp_imm_3;
            iq_rob_id[free_slot_3] <= dp_rob_id_3;
        end

        count <= count
                 + (dispatch_valid[0] & found_0)
                 + (dispatch_valid[1] & found_1)
                 + (dispatch_valid[2] & found_2)
                 + (dispatch_valid[3] & found_3);

        // issue: clear outputs every cycle, only set if something is selected
        issue_valid  <= 1'b0;
        issue_prs1   <= 7'd0;
        issue_prs2   <= 7'd0;
        issue_prd    <= 7'd0;
        issue_opcode <= 7'd0;
        issue_func3  <= 3'd0;
        issue_func7  <= 7'd0;
        issue_imm    <= 32'd0;
        issue_rob_id <= 5'd0;

        if(sel_valid) begin
            issue_valid  <= 1'b1;
            issue_prs1   <= iq_prs1[sel_idx];
            issue_prs2   <= iq_prs2[sel_idx];
            issue_prd    <= iq_prd[sel_idx];
            issue_opcode <= iq_opcode[sel_idx];
            issue_func3  <= iq_func3[sel_idx];
            issue_func7  <= iq_func7[sel_idx];
            issue_imm    <= iq_imm[sel_idx];
            issue_rob_id <= iq_rob_id[sel_idx];
            iq_valid[sel_idx] <= 1'b0;
            count <= count - 1
                     + (dispatch_valid[0] & found_0)
                     + (dispatch_valid[1] & found_1)
                     + (dispatch_valid[2] & found_2)
                     + (dispatch_valid[3] & found_3);
        end

    end
end

endmodule


module Physical_Reg_File(clk, rst,
    alloc_en, alloc_tag_0, alloc_tag_1, alloc_tag_2, alloc_tag_3,
    cdb_valid, cdb_tag, cdb_data,
    rd_tag_0, rd_tag_1,
    rd_data_0, rd_data_1,
    ready_bits);

input clk, rst;
input [3:0] alloc_en;
input [6:0] alloc_tag_0, alloc_tag_1, alloc_tag_2, alloc_tag_3;
input        cdb_valid;
input [6:0]  cdb_tag;
input [63:0] cdb_data;
input  [6:0]  rd_tag_0, rd_tag_1;
output [63:0] rd_data_0, rd_data_1;
output [127:0] ready_bits;

reg [63:0]  prf   [0:127];
reg [127:0] ready;
integer i;

assign ready_bits = ready;
assign rd_data_0  = prf[rd_tag_0];
assign rd_data_1  = prf[rd_tag_1];

always@(posedge clk, negedge rst) begin
    if(!rst) begin
        for(i = 0; i < 128; i = i + 1) begin
            prf[i]   <= 64'd0;
            ready[i] <= 1'b1;
        end
    end
    else begin
        if(cdb_valid) begin
            prf[cdb_tag]   <= cdb_data;
            ready[cdb_tag] <= 1'b1;
        end
        if(alloc_en[0]) ready[alloc_tag_0] <= 1'b0;
        if(alloc_en[1]) ready[alloc_tag_1] <= 1'b0;
        if(alloc_en[2]) ready[alloc_tag_2] <= 1'b0;
        if(alloc_en[3]) ready[alloc_tag_3] <= 1'b0;
    end
end

endmodule

module Arch_Reg_File(clk, rst,
    commit_valid,
    commit_rd_0, commit_rd_1, commit_rd_2, commit_rd_3,
    commit_data_0, commit_data_1, commit_data_2, commit_data_3,
    rd_addr_0, rd_addr_1,
    rd_data_0, rd_data_1);

input clk, rst;
input [3:0] commit_valid;
input [4:0] commit_rd_0, commit_rd_1, commit_rd_2, commit_rd_3;
input [63:0] commit_data_0, commit_data_1, commit_data_2, commit_data_3;
input  [4:0]  rd_addr_0, rd_addr_1;
output [63:0] rd_data_0, rd_data_1;

reg [63:0] arf [0:31];
integer i;

assign rd_data_0 = (rd_addr_0 == 5'd0) ? 64'd0 : arf[rd_addr_0];
assign rd_data_1 = (rd_addr_1 == 5'd0) ? 64'd0 : arf[rd_addr_1];

always@(posedge clk, negedge rst) begin
    if(!rst) begin
        for(i = 0; i < 32; i = i + 1)
            arf[i] <= 64'd0;
    end
    else begin
        if(commit_valid[0] && commit_rd_0 != 5'd0) arf[commit_rd_0] <= commit_data_0;
        if(commit_valid[1] && commit_rd_1 != 5'd0) arf[commit_rd_1] <= commit_data_1;
        if(commit_valid[2] && commit_rd_2 != 5'd0) arf[commit_rd_2] <= commit_data_2;
        if(commit_valid[3] && commit_rd_3 != 5'd0) arf[commit_rd_3] <= commit_data_3;
    end
end

endmodule
