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


`timescale 1ns/1ps
module tb_Arch_Reg_File;

    reg clk, rst;
    reg [3:0]  commit_valid;
    reg [4:0]  commit_rd_0, commit_rd_1, commit_rd_2, commit_rd_3;
    reg [63:0] commit_data_0, commit_data_1, commit_data_2, commit_data_3;
    reg [4:0]  rd_addr_0, rd_addr_1;
    wire [63:0] rd_data_0, rd_data_1;

    Arch_Reg_File dut(
        .clk(clk), .rst(rst),
        .commit_valid(commit_valid),
        .commit_rd_0(commit_rd_0), .commit_rd_1(commit_rd_1),
        .commit_rd_2(commit_rd_2), .commit_rd_3(commit_rd_3),
        .commit_data_0(commit_data_0), .commit_data_1(commit_data_1),
        .commit_data_2(commit_data_2), .commit_data_3(commit_data_3),
        .rd_addr_0(rd_addr_0), .rd_addr_1(rd_addr_1),
        .rd_data_0(rd_data_0), .rd_data_1(rd_data_1)
    );

    always #5 clk = ~clk;

    initial begin
        $dumpfile("arf.vcd");
        $dumpvars(0, tb_Arch_Reg_File);
    end

    initial begin
        clk = 0; rst = 0;
        commit_valid = 0;
        commit_rd_0=0; commit_rd_1=0; commit_rd_2=0; commit_rd_3=0;
        commit_data_0=0; commit_data_1=0; commit_data_2=0; commit_data_3=0;
        rd_addr_0 = 5'd1;
        rd_addr_1 = 5'd3;

        #8; rst = 1;

        // STEP 1: single commit — x1 = 42
        #10;
        commit_valid  = 4'b0001;
        commit_rd_0   = 5'd1;
        commit_data_0 = 64'd42;

        // STEP 2: 4-wide commit — x2=100, x3=200, x4=300, x5=400
        #10;
        commit_valid  = 4'b1111;
        commit_rd_0   = 5'd2;  commit_data_0 = 64'd100;
        commit_rd_1   = 5'd3;  commit_data_1 = 64'd200;
        commit_rd_2   = 5'd4;  commit_data_2 = 64'd300;
        commit_rd_3   = 5'd5;  commit_data_3 = 64'd400;

        // STEP 3: try to write x0 — should be ignored
        #10;
        commit_valid  = 4'b0001;
        commit_rd_0   = 5'd0;
        commit_data_0 = 64'd999;

        #10;
        commit_valid = 0;

        // STEP 4: switch read ports — x0 always 0, x3 = 200
        #5;
        rd_addr_0 = 5'd0;
        rd_addr_1 = 5'd3;

        #20; $finish;
    end

endmodule