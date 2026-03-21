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


`timescale 1ns/1ps
module tb_Issue_Queue;

    reg clk, rst;
    reg [3:0]  dispatch_valid;
    reg [6:0]  dp_prs1_0, dp_prs1_1, dp_prs1_2, dp_prs1_3;
    reg [6:0]  dp_prs2_0, dp_prs2_1, dp_prs2_2, dp_prs2_3;
    reg [6:0]  dp_prd_0,  dp_prd_1,  dp_prd_2,  dp_prd_3;
    reg [6:0]  dp_opcode_0, dp_opcode_1, dp_opcode_2, dp_opcode_3;
    reg [2:0]  dp_func3_0,  dp_func3_1,  dp_func3_2,  dp_func3_3;
    reg [6:0]  dp_func7_0,  dp_func7_1,  dp_func7_2,  dp_func7_3;
    reg [31:0] dp_imm_0,    dp_imm_1,    dp_imm_2,    dp_imm_3;
    reg [4:0]  dp_rob_id_0, dp_rob_id_1, dp_rob_id_2, dp_rob_id_3;
    reg [127:0] prf_ready_bits;
    reg        cdb_valid;
    reg [6:0]  cdb_tag;

    wire        issue_valid;
    wire [6:0]  issue_prs1, issue_prs2, issue_prd;
    wire [6:0]  issue_opcode;
    wire [2:0]  issue_func3;
    wire [6:0]  issue_func7;
    wire [31:0] issue_imm;
    wire [4:0]  issue_rob_id;
    wire        iq_full;

    Issue_Queue dut(
        .clk(clk), .rst(rst),
        .dispatch_valid(dispatch_valid),
        .dp_prs1_0(dp_prs1_0), .dp_prs1_1(dp_prs1_1),
        .dp_prs1_2(dp_prs1_2), .dp_prs1_3(dp_prs1_3),
        .dp_prs2_0(dp_prs2_0), .dp_prs2_1(dp_prs2_1),
        .dp_prs2_2(dp_prs2_2), .dp_prs2_3(dp_prs2_3),
        .dp_prd_0(dp_prd_0),   .dp_prd_1(dp_prd_1),
        .dp_prd_2(dp_prd_2),   .dp_prd_3(dp_prd_3),
        .dp_opcode_0(dp_opcode_0), .dp_opcode_1(dp_opcode_1),
        .dp_opcode_2(dp_opcode_2), .dp_opcode_3(dp_opcode_3),
        .dp_func3_0(dp_func3_0),   .dp_func3_1(dp_func3_1),
        .dp_func3_2(dp_func3_2),   .dp_func3_3(dp_func3_3),
        .dp_func7_0(dp_func7_0),   .dp_func7_1(dp_func7_1),
        .dp_func7_2(dp_func7_2),   .dp_func7_3(dp_func7_3),
        .dp_imm_0(dp_imm_0), .dp_imm_1(dp_imm_1),
        .dp_imm_2(dp_imm_2), .dp_imm_3(dp_imm_3),
        .dp_rob_id_0(dp_rob_id_0), .dp_rob_id_1(dp_rob_id_1),
        .dp_rob_id_2(dp_rob_id_2), .dp_rob_id_3(dp_rob_id_3),
        .prf_ready_bits(prf_ready_bits),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag),
        .issue_valid(issue_valid),
        .issue_prs1(issue_prs1), .issue_prs2(issue_prs2), .issue_prd(issue_prd),
        .issue_opcode(issue_opcode), .issue_func3(issue_func3),
        .issue_func7(issue_func7),   .issue_imm(issue_imm),
        .issue_rob_id(issue_rob_id),
        .iq_full(iq_full)
    );

    always #5 clk = ~clk;

    initial begin
        $dumpfile("iq.vcd");
        $dumpvars(0, tb_Issue_Queue);
    end

    initial begin
        clk = 0; rst = 0;
        dispatch_valid = 0; cdb_valid = 0;
        prf_ready_bits = {128{1'b1}};
        prf_ready_bits[5] = 1'b0;

        dp_prs1_0=0; dp_prs1_1=0; dp_prs1_2=0; dp_prs1_3=0;
        dp_prs2_0=0; dp_prs2_1=0; dp_prs2_2=0; dp_prs2_3=0;
        dp_prd_0=0;  dp_prd_1=0;  dp_prd_2=0;  dp_prd_3=0;
        dp_opcode_0=7'b0110011; dp_opcode_1=7'b0110011;
        dp_opcode_2=7'b0010011; dp_opcode_3=7'b0110011;
        dp_func3_0=0; dp_func3_1=0; dp_func3_2=0; dp_func3_3=0;
        dp_func7_0=0; dp_func7_1=0; dp_func7_2=0; dp_func7_3=0;
        dp_imm_0=0; dp_imm_1=0; dp_imm_2=32'd10; dp_imm_3=0;
        dp_rob_id_0=0; dp_rob_id_1=1; dp_rob_id_2=2; dp_rob_id_3=3;

        #8; rst = 1;

        #10;
        dispatch_valid = 4'b1111;
        dp_prs1_0=7'd2;  dp_prs2_0=7'd3;  dp_prd_0=7'd32;
        dp_prs1_1=7'd5;  dp_prs2_1=7'd6;  dp_prd_1=7'd33;
        dp_prs1_2=7'd4;  dp_prs2_2=7'd0;  dp_prd_2=7'd34;
        dp_prs1_3=7'd5;  dp_prs2_3=7'd7;  dp_prd_3=7'd35;
        #10;
        dispatch_valid = 0;

        #30;

        prf_ready_bits[5] = 1'b1;
        cdb_valid = 1; cdb_tag = 7'd5;
        #10;
        cdb_valid = 0;

        #40; $finish;
    end

endmodule
