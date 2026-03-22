



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


`timescale 1ns/1ps
module tb_LSQ;

    reg clk, rst;
    reg        aq_valid;
    reg        aq_is_load, aq_is_store;
    reg [63:0] aq_addr;
    reg [63:0] aq_store_data;
    reg        aq_store_data_valid;
    reg [6:0]  aq_prd;
    reg [2:0]  aq_func3;
    reg [4:0]  aq_rob_id;
    reg        rob_commit_store;
    reg [4:0]  rob_commit_rob_id;
    wire       cdb_valid;
    wire [6:0] cdb_tag;
    wire [63:0] cdb_data;
    wire       lsq_full;

    LSQ dut(
        .clk(clk), .rst(rst),
        .aq_valid(aq_valid),
        .aq_is_load(aq_is_load), .aq_is_store(aq_is_store),
        .aq_addr(aq_addr),
        .aq_store_data(aq_store_data), .aq_store_data_valid(aq_store_data_valid),
        .aq_prd(aq_prd), .aq_func3(aq_func3), .aq_rob_id(aq_rob_id),
        .rob_commit_store(rob_commit_store),
        .rob_commit_rob_id(rob_commit_rob_id),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_data(cdb_data),
        .lsq_full(lsq_full)
    );

    always #5 clk = ~clk;
    initial begin
        $dumpfile("lsq.vcd");
        $dumpvars(0, tb_LSQ);
    end

    initial begin
        clk = 0; rst = 0;
        aq_valid = 0; rob_commit_store = 0;
        aq_is_load=0; aq_is_store=0;
        aq_addr=0; aq_store_data=0; aq_store_data_valid=0;
        aq_prd=0; aq_func3=0; aq_rob_id=0;
        rob_commit_rob_id=0;

        #8; rst = 1;

        // STEP 1: enqueue STORE — SW x3, 0(x2) → addr=0x200, data=0xDEAD, rob_id=0
        #10;
        aq_valid = 1; aq_is_load=0; aq_is_store=1;
        aq_addr=64'h200; aq_store_data=64'hDEAD;
        aq_store_data_valid=1; aq_prd=7'd0;
        aq_func3=3'd2; aq_rob_id=5'd0;
        #10;
        aq_valid = 0;

        // STEP 2: enqueue LOAD — LW x5, 0(x2) → addr=0x200, prd=p37, rob_id=1
        #10;
        aq_valid = 1; aq_is_load=1; aq_is_store=0;
        aq_addr=64'h200; aq_store_data=0;
        aq_store_data_valid=0; aq_prd=7'd37;
        aq_func3=3'd2; aq_rob_id=5'd1;
        #10;
        aq_valid = 0;

        // STEP 3: ROB commits the store (rob_id=0) → data_mem[0x40] = 0xDEAD
        #10;
        rob_commit_store    = 1;
        rob_commit_rob_id   = 5'd0;
        #10;
        rob_commit_store = 0;
        // load now executes → cdb_valid=1, cdb_tag=37, cdb_data=0xDEAD

        #40; $finish;
    end

endmodule
