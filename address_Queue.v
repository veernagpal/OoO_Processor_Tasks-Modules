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


`timescale 1ns/1ps
module tb_Address_Queue;

    reg clk, rst;
    reg [3:0]  dispatch_valid;
    reg dp_is_load_0, dp_is_load_1, dp_is_load_2, dp_is_load_3;
    reg dp_is_store_0, dp_is_store_1, dp_is_store_2, dp_is_store_3;
    reg [6:0]  dp_prs1_0, dp_prs1_1, dp_prs1_2, dp_prs1_3;
    reg [6:0]  dp_prs2_0, dp_prs2_1, dp_prs2_2, dp_prs2_3;
    reg [6:0]  dp_prd_0,  dp_prd_1,  dp_prd_2,  dp_prd_3;
    reg [31:0] dp_imm_0,  dp_imm_1,  dp_imm_2,  dp_imm_3;
    reg [2:0]  dp_func3_0, dp_func3_1, dp_func3_2, dp_func3_3;
    reg [4:0]  dp_rob_id_0, dp_rob_id_1, dp_rob_id_2, dp_rob_id_3;
    reg [127:0] prf_ready_bits;
    reg        cdb_valid;
    reg [6:0]  cdb_tag;
    reg [63:0] cdb_data;

    wire        lsq_addr_valid, lsq_is_load, lsq_is_store, lsq_store_data_valid;
    wire [63:0] lsq_addr, lsq_store_data;
    wire [6:0]  lsq_prd;
    wire [2:0]  lsq_func3;
    wire [4:0]  lsq_rob_id;
    wire        aq_full;

    Address_Queue dut(
        .clk(clk), .rst(rst),
        .dispatch_valid(dispatch_valid),
        .dp_is_load_0(dp_is_load_0),   .dp_is_load_1(dp_is_load_1),
        .dp_is_load_2(dp_is_load_2),   .dp_is_load_3(dp_is_load_3),
        .dp_is_store_0(dp_is_store_0), .dp_is_store_1(dp_is_store_1),
        .dp_is_store_2(dp_is_store_2), .dp_is_store_3(dp_is_store_3),
        .dp_prs1_0(dp_prs1_0), .dp_prs1_1(dp_prs1_1),
        .dp_prs1_2(dp_prs1_2), .dp_prs1_3(dp_prs1_3),
        .dp_prs2_0(dp_prs2_0), .dp_prs2_1(dp_prs2_1),
        .dp_prs2_2(dp_prs2_2), .dp_prs2_3(dp_prs2_3),
        .dp_prd_0(dp_prd_0), .dp_prd_1(dp_prd_1),
        .dp_prd_2(dp_prd_2), .dp_prd_3(dp_prd_3),
        .dp_imm_0(dp_imm_0), .dp_imm_1(dp_imm_1),
        .dp_imm_2(dp_imm_2), .dp_imm_3(dp_imm_3),
        .dp_func3_0(dp_func3_0), .dp_func3_1(dp_func3_1),
        .dp_func3_2(dp_func3_2), .dp_func3_3(dp_func3_3),
        .dp_rob_id_0(dp_rob_id_0), .dp_rob_id_1(dp_rob_id_1),
        .dp_rob_id_2(dp_rob_id_2), .dp_rob_id_3(dp_rob_id_3),
        .prf_ready_bits(prf_ready_bits),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_data(cdb_data),
        .lsq_addr_valid(lsq_addr_valid),
        .lsq_is_load(lsq_is_load), .lsq_is_store(lsq_is_store),
        .lsq_addr(lsq_addr),
        .lsq_store_data(lsq_store_data), .lsq_store_data_valid(lsq_store_data_valid),
        .lsq_prd(lsq_prd), .lsq_func3(lsq_func3), .lsq_rob_id(lsq_rob_id),
        .aq_full(aq_full)
    );

    always #5 clk = ~clk;
    initial begin
        $dumpfile("aq.vcd");
        $dumpvars(0, tb_Address_Queue);
    end

    initial begin
        clk = 0; rst = 0;
        dispatch_valid = 0; cdb_valid = 0;
        prf_ready_bits = {128{1'b1}};
        prf_ready_bits[2] = 1'b0;
        dp_is_load_0=0; dp_is_load_1=0; dp_is_load_2=0; dp_is_load_3=0;
        dp_is_store_0=0; dp_is_store_1=0; dp_is_store_2=0; dp_is_store_3=0;
        dp_prs1_0=0; dp_prs2_0=0; dp_prd_0=0; dp_imm_0=0; dp_func3_0=0; dp_rob_id_0=0;
        dp_prs1_1=0; dp_prs2_1=0; dp_prd_1=0; dp_imm_1=0; dp_func3_1=0; dp_rob_id_1=0;
        dp_prs1_2=0; dp_prs2_2=0; dp_prd_2=0; dp_imm_2=0; dp_func3_2=0; dp_rob_id_2=0;
        dp_prs1_3=0; dp_prs2_3=0; dp_prd_3=0; dp_imm_3=0; dp_func3_3=0; dp_rob_id_3=0;
        cdb_tag=0; cdb_data=0;

        #8; rst = 1;

        #10;
        dispatch_valid = 4'b0001;
        dp_is_load_0   = 1;
        dp_is_store_0  = 0;
        dp_prs1_0      = 7'd2;
        dp_prs2_0      = 7'd0;
        dp_prd_0       = 7'd37;
        dp_imm_0       = 32'd8;
        dp_func3_0     = 3'd2;
        dp_rob_id_0    = 5'd1;
        #10;
        dispatch_valid = 0;

        #20;
        cdb_valid = 1; cdb_tag = 7'd2; cdb_data = 64'h100;
        #10;
        cdb_valid = 0;

        #30; $finish;
    end

endmodule
