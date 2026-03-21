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


`timescale 1ns/1ps
module tb_ROB;

    reg clk, rst;
    reg [3:0] dispatch_valid;
    reg [4:0] dispatch_rd_0,  dispatch_rd_1,  dispatch_rd_2,  dispatch_rd_3;
    reg [6:0] dispatch_prd_0, dispatch_prd_1, dispatch_prd_2, dispatch_prd_3;
    reg [6:0] dispatch_old_prd_0, dispatch_old_prd_1, dispatch_old_prd_2, dispatch_old_prd_3;
    reg [3:0] dispatch_has_dest, dispatch_is_store;
    wire [4:0] rob_id_0, rob_id_1, rob_id_2, rob_id_3;
    wire       rob_full;
    reg        cdb_valid;
    reg [4:0]  cdb_rob_id;
    wire [3:0] commit_valid;
    wire [4:0] commit_rd_0, commit_rd_1, commit_rd_2, commit_rd_3;
    wire [6:0] commit_prd_0, commit_prd_1, commit_prd_2, commit_prd_3;
    wire [6:0] commit_old_prd_0, commit_old_prd_1, commit_old_prd_2, commit_old_prd_3;
    wire [3:0] commit_has_dest, commit_is_store;

    ROB dut(
        .clk(clk), .rst(rst),
        .dispatch_valid(dispatch_valid),
        .dispatch_rd_0(dispatch_rd_0), .dispatch_rd_1(dispatch_rd_1),
        .dispatch_rd_2(dispatch_rd_2), .dispatch_rd_3(dispatch_rd_3),
        .dispatch_prd_0(dispatch_prd_0), .dispatch_prd_1(dispatch_prd_1),
        .dispatch_prd_2(dispatch_prd_2), .dispatch_prd_3(dispatch_prd_3),
        .dispatch_old_prd_0(dispatch_old_prd_0), .dispatch_old_prd_1(dispatch_old_prd_1),
        .dispatch_old_prd_2(dispatch_old_prd_2), .dispatch_old_prd_3(dispatch_old_prd_3),
        .dispatch_has_dest(dispatch_has_dest), .dispatch_is_store(dispatch_is_store),
        .rob_id_0(rob_id_0), .rob_id_1(rob_id_1),
        .rob_id_2(rob_id_2), .rob_id_3(rob_id_3),
        .rob_full(rob_full),
        .cdb_valid(cdb_valid), .cdb_rob_id(cdb_rob_id),
        .commit_valid(commit_valid),
        .commit_rd_0(commit_rd_0), .commit_rd_1(commit_rd_1),
        .commit_rd_2(commit_rd_2), .commit_rd_3(commit_rd_3),
        .commit_prd_0(commit_prd_0), .commit_prd_1(commit_prd_1),
        .commit_prd_2(commit_prd_2), .commit_prd_3(commit_prd_3),
        .commit_old_prd_0(commit_old_prd_0), .commit_old_prd_1(commit_old_prd_1),
        .commit_old_prd_2(commit_old_prd_2), .commit_old_prd_3(commit_old_prd_3),
        .commit_has_dest(commit_has_dest), .commit_is_store(commit_is_store)
    );

    always #5 clk = ~clk;

    initial begin
        $dumpfile("rob.vcd");
        $dumpvars(0, tb_ROB);
    end

    initial begin
        clk = 0; rst = 0;
        dispatch_valid = 0; cdb_valid = 0;
        dispatch_has_dest = 4'b1111; dispatch_is_store = 4'b0000;
        #8; rst = 1;

        #10;
        dispatch_valid   = 4'b1111;
        dispatch_rd_0    = 5'd1;  dispatch_prd_0    = 7'd32; dispatch_old_prd_0 = 7'd1;
        dispatch_rd_1    = 5'd2;  dispatch_prd_1    = 7'd33; dispatch_old_prd_1 = 7'd2;
        dispatch_rd_2    = 5'd3;  dispatch_prd_2    = 7'd34; dispatch_old_prd_2 = 7'd3;
        dispatch_rd_3    = 5'd4;  dispatch_prd_3    = 7'd35; dispatch_old_prd_3 = 7'd4;
        #10;
        dispatch_valid = 0;

        // STEP 2: CDB marks ROB entry 2 done (out of order — instruction 2 finishes first)
        #10;
        cdb_valid = 1; cdb_rob_id = 5'd2;
        #10;
        cdb_valid = 0;
        // NOTE: commit_valid should still be 0 — entry 0 (head) not done yet

        // STEP 3: CDB marks ROB entry 0 done — head can now commit
        #10;
        cdb_valid = 1; cdb_rob_id = 5'd0;
        #10;
        cdb_valid = 0;
        // commit_valid[0] should fire — commit x1/p32

        // STEP 4: CDB marks entries 1 and 3 done
        #10;
        cdb_valid = 1; cdb_rob_id = 5'd1;
        #10;
        cdb_valid = 0;
        #10;
        cdb_valid = 1; cdb_rob_id = 5'd3;
        #10;
        cdb_valid = 0;
        // entries 1,2,3 all done now → should commit in order

        #40; $finish;
    end

endmodule
