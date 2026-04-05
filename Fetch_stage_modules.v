module Fetch_Stage(
    input        clk, reset,
    input        read_en,
    input  [2:0] read_count,
    output [31:0] out0, out1, out2, out3,
    output [2:0]  write_count
);

    wire [63:0] current_pc;
    wire [31:0] next_pc_32;
    wire [63:0] next_pc_64;
    assign next_pc_64 = {32'd0, next_pc_32};

    wire [31:0] imem_out0, imem_out1, imem_out2, imem_out3;
    wire [31:0] pd_out0, pd_out1, pd_out2, pd_out3;

    PC pc_reg(
        .clk(clk), .reset(reset),
        .next_pc(next_pc_64),
        .current_pc(current_pc)
    );

    instruction_memory imem(
        .pc_input_address(current_pc),
        .instn_out0(imem_out0), .instn_out1(imem_out1),
        .instn_out2(imem_out2), .instn_out3(imem_out3)
    );

    predecode pd(
        .pc(current_pc[31:0]),
        .ins0(imem_out0), .ins1(imem_out1),
        .ins2(imem_out2), .ins3(imem_out3),
        .write_count(write_count),
        .next_pc(next_pc_32),
        .inst_out0(pd_out0), .inst_out1(pd_out1),
        .inst_out2(pd_out2), .inst_out3(pd_out3)
    );

    instn_queue iq(
        .clk(clk), .reset(reset),
        .inst0(pd_out0), .inst1(pd_out1),
        .inst2(pd_out2), .inst3(pd_out3),
        .write_en(1'b1),
        .write_count(write_count),
        .read_en(read_en),
        .read_count(read_count),
        .out0(out0), .out1(out1),
        .out2(out2), .out3(out3)
    );

endmodule






module PC(clk, next_pc, current_pc, reset);
    input clk, reset;
    input  [63:0] next_pc;
    output reg [63:0] current_pc;

    always@(posedge clk)begin
        if(reset)
            current_pc <= 0;
        else
            current_pc <= next_pc;
    end
endmodule


module PC_4_wide_fetch_adder(pc_input, pc_incremented_normal);
    input  [63:0] pc_input;
    output [63:0] pc_incremented_normal;
    assign pc_incremented_normal = pc_input + 16;
endmodule


module instruction_memory(pc_input_address, instn_out0, instn_out1, instn_out2, instn_out3);
    input  [63:0] pc_input_address;
    output [31:0] instn_out0, instn_out1, instn_out2, instn_out3;
    reg [31:0] instn_array [0:63];

    initial begin
        // --- PC=0: ADD, ADDI, BEQ→32, ADDI ---
        instn_array[0]  = 32'h00000033; // ADD
        instn_array[1]  = 32'h00000013; // ADDI
        instn_array[2]  = 32'h00000C63; // BEQ, imm=24, target = 0+8+24 = 32
        instn_array[3]  = 32'h00000013; // ADDI (after branch — predecode discards)

        // --- (PC=16 never fetched in our sequence) ---
        instn_array[4]  = 32'h00000033;
        instn_array[5]  = 32'h00000013;
        instn_array[6]  = 32'h00000013;
        instn_array[7]  = 32'h00000033;

        // --- PC=32: ADD, ADDI, ADDI, ADD (no branch) ---
        instn_array[8]  = 32'h00000033; // ADD
        instn_array[9]  = 32'h00000013; // ADDI
        instn_array[10] = 32'h00000013; // ADDI
        instn_array[11] = 32'h00000033; // ADD

        // --- PC=48: ADDI, BNE→64, ADD, ADDI ---
        instn_array[12] = 32'h00000013; // ADDI
        instn_array[13] = 32'h00001663; // BNE, imm=12, target = 48+4+12 = 64
        instn_array[14] = 32'h00000033; // ADD  (after branch — predecode discards)
        instn_array[15] = 32'h00000013; // ADDI (after branch — predecode discards)

        // --- PC=64: ADD, ADDI, ADD, BLT→80 ---
        instn_array[16] = 32'h00000033; // ADD
        instn_array[17] = 32'h00000013; // ADDI
        instn_array[18] = 32'h00000033; // ADD
        instn_array[19] = 32'h00004263; // BLT, imm=4, target = 64+12+4 = 80

        // remaining entries zero
        begin : zero_fill
            integer j;
            for(j = 20; j < 64; j = j+1)
                instn_array[j] = 32'h00000013; // ADDI (NOP-like)
        end
    end

    // pc[7:2] divides byte address by 4 to get word index
    wire [5:0] base_index;
    assign base_index = pc_input_address[7:2];

    assign instn_out0 = instn_array[base_index];
    assign instn_out1 = instn_array[base_index + 1];
    assign instn_out2 = instn_array[base_index + 2];
    assign instn_out3 = instn_array[base_index + 3];
endmodule


module instn_queue(clk, reset, inst0, inst1, inst2, inst3,
                   out0, out1, out2, out3,
                   write_en, write_count, read_count, read_en);

    input clk, reset, write_en, read_en;
    input [31:0] inst0, inst1, inst2, inst3;
    input [2:0] write_count, read_count;
    output reg [31:0] out0, out1, out2, out3;

    reg [31:0] fifo_mem [0:31];
    reg [4:0] head;
    reg [4:0] tail;

    always@(posedge clk)begin
        if(reset)begin
            head <= 0;
            tail <= 0;
        end
        else begin
            if(write_en)begin
                if(write_count == 1)begin
                    fifo_mem[tail] <= inst0;
                end
                else if(write_count == 2)begin
                    fifo_mem[tail]     <= inst0;
                    fifo_mem[tail + 1] <= inst1;
                end
                else if(write_count == 3)begin
                    fifo_mem[tail]     <= inst0;
                    fifo_mem[tail + 1] <= inst1;
                    fifo_mem[tail + 2] <= inst2;
                end
                else begin
                    fifo_mem[tail]     <= inst0;
                    fifo_mem[tail + 1] <= inst1;
                    fifo_mem[tail + 2] <= inst2;
                    fifo_mem[tail + 3] <= inst3;
                end
                tail <= tail + write_count;
            end

            if(read_en)begin
                if(read_count == 1)begin
                    out0 <= fifo_mem[head];
                end
                else if(read_count == 2)begin
                    out0 <= fifo_mem[head];
                    out1 <= fifo_mem[head + 1];
                end
                else if(read_count == 3)begin
                    out0 <= fifo_mem[head];
                    out1 <= fifo_mem[head + 1];
                    out2 <= fifo_mem[head + 2];
                end
                else begin
                    out0 <= fifo_mem[head];
                    out1 <= fifo_mem[head + 1];
                    out2 <= fifo_mem[head + 2];
                    out3 <= fifo_mem[head + 3];
                end
                head <= head + read_count;
            end
        end
    end
endmodule

module predecode(
    input  [31:0] pc,
    input  [31:0] ins0, ins1, ins2, ins3,
    output reg [2:0]  write_count,
    output reg [31:0] next_pc,
    output reg [31:0] inst_out0, inst_out1, inst_out2, inst_out3
);

    wire [6:0] op0 = ins0[6:0];
    wire [6:0] op1 = ins1[6:0];
    wire [6:0] op2 = ins2[6:0];
    wire [6:0] op3 = ins3[6:0];

    wire br0  = (op0 == 7'b1100011);
    wire br1  = (op1 == 7'b1100011);
    wire br2  = (op2 == 7'b1100011);
    wire br3  = (op3 == 7'b1100011);
    wire jal0 = (op0 == 7'b1101111);
    wire jal1 = (op1 == 7'b1101111);
    wire jal2 = (op2 == 7'b1101111);
    wire jal3 = (op3 == 7'b1101111);

    // B-type immediate extraction
    wire [12:0] br_imm0 = {ins0[31], ins0[7], ins0[30:25], ins0[11:8], 1'b0};
    wire [12:0] br_imm1 = {ins1[31], ins1[7], ins1[30:25], ins1[11:8], 1'b0};
    wire [12:0] br_imm2 = {ins2[31], ins2[7], ins2[30:25], ins2[11:8], 1'b0};
    wire [12:0] br_imm3 = {ins3[31], ins3[7], ins3[30:25], ins3[11:8], 1'b0};

    wire [31:0] br_target0 = pc +  0 + {{19{br_imm0[12]}}, br_imm0};
    wire [31:0] br_target1 = pc +  4 + {{19{br_imm1[12]}}, br_imm1};
    wire [31:0] br_target2 = pc +  8 + {{19{br_imm2[12]}}, br_imm2};
    wire [31:0] br_target3 = pc + 12 + {{19{br_imm3[12]}}, br_imm3};

    // J-type immediate extraction
    wire [20:0] jal_imm0 = {ins0[31], ins0[19:12], ins0[20], ins0[30:21], 1'b0};
    wire [20:0] jal_imm1 = {ins1[31], ins1[19:12], ins1[20], ins1[30:21], 1'b0};
    wire [20:0] jal_imm2 = {ins2[31], ins2[19:12], ins2[20], ins2[30:21], 1'b0};
    wire [20:0] jal_imm3 = {ins3[31], ins3[19:12], ins3[20], ins3[30:21], 1'b0};

    wire [31:0] jal_target0 = pc +  0 + {{11{jal_imm0[20]}}, jal_imm0};
    wire [31:0] jal_target1 = pc +  4 + {{11{jal_imm1[20]}}, jal_imm1};
    wire [31:0] jal_target2 = pc +  8 + {{11{jal_imm2[20]}}, jal_imm2};
    wire [31:0] jal_target3 = pc + 12 + {{11{jal_imm3[20]}}, jal_imm3};

    always@(*)begin
        if(br0 || jal0)begin
            write_count = 3'd1;
            next_pc     = br0 ? br_target0 : jal_target0;
            inst_out0 = ins0; inst_out1 = 0; inst_out2 = 0; inst_out3 = 0;
        end
        else if(br1 || jal1)begin
            write_count = 3'd2;
            next_pc     = br1 ? br_target1 : jal_target1;
            inst_out0 = ins0; inst_out1 = ins1; inst_out2 = 0; inst_out3 = 0;
        end
        else if(br2 || jal2)begin
            write_count = 3'd3;
            next_pc     = br2 ? br_target2 : jal_target2;
            inst_out0 = ins0; inst_out1 = ins1; inst_out2 = ins2; inst_out3 = 0;
        end
        else if(br3 || jal3)begin
            write_count = 3'd4;
            next_pc     = br3 ? br_target3 : jal_target3;
            inst_out0 = ins0; inst_out1 = ins1; inst_out2 = ins2; inst_out3 = ins3;
        end
        else begin
            write_count = 3'd4;
            next_pc     = pc + 32'd16;
            inst_out0 = ins0; inst_out1 = ins1; inst_out2 = ins2; inst_out3 = ins3;
        end
    end
endmodule


`timescale 1ns/1ps

module tb_Fetch_Stage;

    reg        clk, reset;
    reg        read_en;
    reg [2:0]  read_count;

    wire [31:0] out0, out1, out2, out3;
    wire [2:0]  write_count;

    // tap internal signals for waveform visibility
    wire [63:0] current_pc;
    wire [31:0] imem_out0, imem_out1, imem_out2, imem_out3;
    wire [31:0] next_pc_32;

    Fetch_Stage dut(
        .clk(clk), .reset(reset),
        .read_en(read_en), .read_count(read_count),
        .out0(out0), .out1(out1), .out2(out2), .out3(out3),
        .write_count(write_count)
    );

    assign current_pc  = dut.current_pc;
    assign imem_out0   = dut.imem_out0;
    assign imem_out1   = dut.imem_out1;
    assign imem_out2   = dut.imem_out2;
    assign imem_out3   = dut.imem_out3;
    assign next_pc_32  = dut.next_pc_32;

    always #5 clk = ~clk;

    initial begin
        $dumpfile("tb_fetch_stage.vcd");
        $dumpvars(0, tb_Fetch_Stage);
    end

    initial begin
        clk        = 0;
        reset      = 1;
        read_en    = 0;
        read_count = 0;

        // hold reset for 2 cycles so PC and FIFO are cleanly initialised
        #20; reset = 0;

        // let fetch run for 4 cycles — populates the FIFO with 3+4+2+4=13 instructions
        #40;

        // READ 1: read 3 instructions (bundle from PC=0, stops at BEQ)
        // expect: out0=0x33(ADD), out1=0x13(ADDI), out2=0xC63(BEQ)
        read_en    = 1;
        read_count = 3'd3;
        #10;
        read_en = 0;

        // wait 2 cycles (more fetches keep happening)
        #20;

        // READ 2: read 4 instructions (bundle from PC=32, no branch)
        // expect: out0=0x33(ADD), out1=0x13(ADDI), out2=0x13(ADDI), out3=0x33(ADD)
        read_en    = 1;
        read_count = 3'd4;
        #10;
        read_en = 0;

        #40;
        $finish;
    end

endmodule
