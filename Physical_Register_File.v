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


//testbench
`timescale 1ns/1ps
module tb_Physical_Reg_File;

    reg clk, rst;
    reg [3:0]  alloc_en;
    reg [6:0]  alloc_tag_0, alloc_tag_1, alloc_tag_2, alloc_tag_3;
    reg        cdb_valid;
    reg [6:0]  cdb_tag;
    reg [63:0] cdb_data;
    reg [6:0]  rd_tag_0, rd_tag_1;
    wire [63:0] rd_data_0, rd_data_1;
    wire [127:0] ready_bits;

    Physical_Reg_File dut(
        .clk(clk), .rst(rst),
        .alloc_en(alloc_en),
        .alloc_tag_0(alloc_tag_0), .alloc_tag_1(alloc_tag_1),
        .alloc_tag_2(alloc_tag_2), .alloc_tag_3(alloc_tag_3),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_data(cdb_data),
        .rd_tag_0(rd_tag_0), .rd_tag_1(rd_tag_1),
        .rd_data_0(rd_data_0), .rd_data_1(rd_data_1),
        .ready_bits(ready_bits)
    );

    always #5 clk = ~clk;

    initial begin
        $dumpfile("prf.vcd");
        $dumpvars(0, tb_Physical_Reg_File);
    end

    initial begin
        clk       = 0;
        rst       = 0;        // reset active
        alloc_en  = 4'b0000;
        cdb_valid = 0;
        alloc_tag_0 = 7'd0; alloc_tag_1 = 7'd0;
        alloc_tag_2 = 7'd0; alloc_tag_3 = 7'd0;
        cdb_tag   = 7'd0;
        cdb_data  = 64'd0;

        rd_tag_0  = 7'd32;
        rd_tag_1  = 7'd33;

        // t=8: release reset
        #8; rst = 1;
 
        #10;                        // t=18
        alloc_en    = 4'b0011;
        alloc_tag_0 = 7'd32;
        alloc_tag_1 = 7'd33;

        #10;                        // t=28
        alloc_en = 4'b0000;

        #10;                        // t=38
        cdb_valid = 1;
        cdb_tag   = 7'd32;
        cdb_data  = 64'd100;

        #10;                        // t=48
        cdb_valid = 0;

    -
        #10;                        // t=58
        cdb_valid = 1;
        cdb_tag   = 7'd33;
        cdb_data  = 64'd200;

        #10;                        // t=68
        cdb_valid = 0;


        #10;                        // t=78
        alloc_en    = 4'b0001;
        alloc_tag_0 = 7'd34;

        #10;                        // t=88
        alloc_en = 4'b0000;

        #30; $finish;
    end

endmodule
