
module PC(clk , next_pc , current_pc , reset);
input clk , reset;
input [63:0] next_pc;
output reg [63:0] current_pc;
always @ (posedge clk) begin
    
    if(reset) //active high synchronous reset
    current_pc <= 0;

    else
    current_pc <= next_pc;

end

endmodule

`timescale 1ns/1ps

module tb_PC;

    reg clk;
    reg reset;
    reg [63:0] next_pc;
    wire [63:0] current_pc;

    // Instantiate your PC
    PC uut (
        .clk(clk),
        .reset(reset),
        .next_pc(next_pc),
        .current_pc(current_pc)
    );

    // Clock generation: 10ns period
    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        // Dump for GTKWave
        $dumpfile("tb_PC.vcd");
        $dumpvars(0, tb_PC);

        // Initialize signals
        reset = 1;
        next_pc = 0;

        // Apply reset for 1 cycle
        #10;
        reset = 0;

        // Feed some next_pc values
        next_pc = 64'd4;  // PC should update to 4 on next clock
        #10;              // wait 1 clock
        next_pc = 64'd8;  // PC should update to 8
        #10;
        next_pc = 64'd16; // PC should update to 16
        #10;
        next_pc = 64'd32; // PC should update to 32
        #10;

        // Assert reset again
        reset = 1;
        #10;
        reset = 0;

        next_pc = 64'd100;
        #10;

        // Finish simulation
        $finish;
    end

endmodule


module PC_4_wide_fetch_adder(pc_input , pc_incremented_normal); // this PC adder facilitates 4 CONSECUTIVE instructions being fetched at once
input [63:0] pc_input;
output[63:0] pc_incremented_normal; //this will be one of the inputs to the PC_mux
assign pc_incremented_normal = pc_input + 16;
endmodule



module instruction_memory(pc_input_address , instn_out0 , instn_out1 , instn_out2 , instn_out3);
input [63:0] pc_input_address;
output [31:0] instn_out0 , instn_out1 , instn_out2 , instn_out3;
reg [31:0] instn_array [63:0] ;


initial begin
    instn_array[0]  = 32'b0000000_00000_00000_000_00000_0110011; // ADD   | hex: 0x00000033 | dec: 51
    instn_array[1]  = 32'b0000000_00000_00000_000_00000_0010011; // ADDI  | hex: 0x00000013 | dec: 19
    instn_array[2]  = 32'b0000000_00000_00000_000_00000_1100011; // BEQ   | hex: 0x00000063 | dec: 99
    instn_array[3]  = 32'b0000000_00000_00000_000_00000_0010011; // ADDI  | hex: 0x00000013 | dec: 19

    instn_array[4]  = 32'b0000000_00000_00000_000_00000_0110011; // ADD   | hex: 0x00000033 | dec: 51
    instn_array[5]  = 32'b0000000_00000_00000_001_00000_1100011; // BNE   | hex: 0x00001063 | dec: 4195
    instn_array[6]  = 32'b0000000_00000_00000_000_00000_0010011; // ADDI  | hex: 0x00000013 | dec: 19
    instn_array[7]  = 32'b0000000_00000_00000_000_00000_0110011; // ADD   | hex: 0x00000033 | dec: 51

    instn_array[8]  = 32'b0000000_00000_00000_000_00000_0110011; // ADD   | hex: 0x00000033 | dec: 51
    instn_array[9]  = 32'b0000000_00000_00000_000_00000_0010011; // ADDI  | hex: 0x00000013 | dec: 19
    instn_array[10] = 32'b0000000_00000_00000_100_00000_1100011; // BLT   | hex: 0x00004063 | dec: 16483
    instn_array[11] = 32'b0000000_00000_00000_000_00000_0110011; // ADD   | hex: 0x00000033 | dec: 51

    instn_array[12] = 32'b0000000_00000_00000_000_00000_0010011; // ADDI  | hex: 0x00000013 | dec: 19
    instn_array[13] = 32'b0000000_00000_00000_110_00000_1100011; // BLTU  | hex: 0x00006063 | dec: 24675
    instn_array[14] = 32'b0000000_00000_00000_000_00000_0110011; // ADD   | hex: 0x00000033 | dec: 51
    instn_array[15] = 32'b0000000_00000_00000_000_00000_0010011; // ADDI  | hex: 0x00000013 | dec: 19

    instn_array[16] = 32'b0000000_00000_00000_000_00000_0110011; // ADD   | hex: 0x00000033 | dec: 51
    instn_array[17] = 32'b0000000_00000_00000_111_00000_1100011; // BGEU  | hex: 0x00007063 | dec: 28771
    instn_array[18] = 32'b0000000_00000_00000_000_00000_0010011; // ADDI  | hex: 0x00000013 | dec: 19
    instn_array[19] = 32'b0000000_00000_00000_000_00000_0110011; // ADD   | hex: 0x00000033 | dec: 51


end

wire [5:0] pc_divide_base_index; // 6 bits to address 64 instruction memory locations
assign pc_divide_base_index = pc_input_address[7:2]; //dividing the PC by 4 to ensure indexing matches with the indexing of the instruction memory
                                                    //can be extended to point to how many ever indexes in the instruction memory within the range 
assign instn_out0 = instn_array [pc_divide_base_index]; 
assign instn_out1 = instn_array [pc_divide_base_index + 1]; 
assign instn_out2 = instn_array [pc_divide_base_index + 2]; 
assign instn_out3 = instn_array [pc_divide_base_index + 3]; 

endmodule

`timescale 1ns/1ps

module tb_instruction_memory;

    reg  [63:0] pc_input_address;
    wire [31:0] inst0, inst1, inst2, inst3;

    // Instantiate instruction memory
    instruction_memory uut (
        .pc_input_address(pc_input_address),
        .instn_out0(inst0),
        .instn_out1(inst1),
        .instn_out2(inst2),
        .instn_out3(inst3)
    );

    initial begin
        // GTKWave dump
        $dumpfile("tb_instruction_memory.vcd");
        $dumpvars(0, tb_instruction_memory);

        // Start PC at 0
        pc_input_address = 64'd0;

        // Fetch 4-wide bundles
        #10 pc_input_address = 64'd16;   // inst[4..7]
        #10 pc_input_address = 64'd32;   // inst[8..11]
        #10 pc_input_address = 64'd48;   // inst[12..15]
        #10 pc_input_address = 64'd64;   // inst[16..19] (undefined unless initialized)

        #10 $finish;
    end

endmodule


module instn_queue(clk,reset,inst0,inst1,inst2,inst3,out0,out1,out2,out3,write_en,write_count,read_count,read_en); 

//this module is a synchronous FIFO which takes in parallel the instructions from the instruction memory upto (including) 
//the first branch statement in the same order, and the read will also happen in a parallel manner, i.e all instructions 
//can be read at once, both the read and write operations to this FIFO are clocked  

input clk,reset,write_en,read_en;
input [31:0] inst0,inst1,inst2,inst3;
input [2:0] write_count,read_count; // upto which instruction we should write/read to the instruction queue (inclusive of the first branch statement encountered)
output reg [31:0] out0,out1,out2,out3;

reg [31:0] fifo_mem [0:31];
reg [4:0] head;
reg [4:0] tail;

always @(posedge clk) begin
    if (reset) begin
        head <= 0;
        tail <= 0;
    end else begin

        if (write_en) begin
            if (write_count == 1) begin
                fifo_mem[tail] <= inst0;
            end
            else if (write_count == 2) begin
                fifo_mem[tail]     <= inst0;
                fifo_mem[tail + 1] <= inst1;
            end
            else if (write_count == 3) begin
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

        if (read_en) begin
            if (read_count == 1) begin
                out0 <= fifo_mem[head];
            end
            else if (read_count == 2) begin
                out0 <= fifo_mem[head];
                out1 <= fifo_mem[head + 1];
            end
            else if (read_count == 3) begin
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

`timescale 1ns/1ps

module tb_instn_queue;

    reg clk;
    reg reset;

    reg write_en, read_en;
    reg [2:0] write_count, read_count;
    reg [31:0] inst0, inst1, inst2, inst3;

    wire [31:0] out0, out1, out2, out3;

    instn_queue dut (
        .clk(clk),
        .reset(reset),
        .inst0(inst0),
        .inst1(inst1),
        .inst2(inst2),
        .inst3(inst3),
        .out0(out0),
        .out1(out1),
        .out2(out2),
        .out3(out3),
        .write_en(write_en),
        .write_count(write_count),
        .read_count(read_count),
        .read_en(read_en)
    );

    // clock
    always #5 clk = ~clk;

    initial begin
        $dumpfile("fifo_tb.vcd");
        $dumpvars(0, tb_instn_queue);
    end

    initial begin
        clk = 0;
        reset = 1;
        write_en = 0;
        read_en  = 0;
        write_count = 0;
        read_count  = 0;

        inst0 = 0; inst1 = 0; inst2 = 0; inst3 = 0;

        #10;
        reset = 0;


        #10;
        write_en = 1;
        write_count = 2;
        inst0 = 32'hAAAA0000;
        inst1 = 32'hBBBB0000;

        #10;
        write_en = 0;


        #10;
        write_en = 1;
        write_count = 3;
        inst0 = 32'h11110000;
        inst1 = 32'h22220000;
        inst2 = 32'h33330000;

        #10;
        write_en = 0;


        #10;
        read_en = 1;
        read_count = 2;

        #10;
        read_en = 0;


        #10;
        read_en = 1;
        read_count = 3;

        #10;
        read_en = 0;

        #50;
        $finish;
    end

endmodule




module predecode(
    input  [31:0] pc,            // PC of first instruction
    input  [31:0] ins0, ins1, ins2, ins3,  // 4 instructions from IMEM
    output reg [2:0] write_count, // how many instructions FIFO should enqueue
    output reg [31:0] next_pc ,    // next PC to fetch
    output reg [31:0] inst_out0 , inst_out1 , inst_out2 , inst_out3);

    // Extract opcodes
    wire [6:0] op0 = ins0[6:0];
    wire [6:0] op1 = ins1[6:0];
    wire [6:0] op2 = ins2[6:0];
    wire [6:0] op3 = ins3[6:0];

    // Detect branches and JAL
    wire br0 = (op0 == 7'b1100011);
    wire br1 = (op1 == 7'b1100011);
    wire br2 = (op2 == 7'b1100011);
    wire br3 = (op3 == 7'b1100011);

    wire jal0 = (op0 == 7'b1101111);
    wire jal1 = (op1 == 7'b1101111);
    wire jal2 = (op2 == 7'b1101111);
    wire jal3 = (op3 == 7'b1101111);

    // Compute branch offsets (B-type)
    wire [12:0] br_imm0 = {ins0[31], ins0[7], ins0[30:25], ins0[11:8], 1'b0};
    wire [12:0] br_imm1 = {ins1[31], ins1[7], ins1[30:25], ins1[11:8], 1'b0};
    wire [12:0] br_imm2 = {ins2[31], ins2[7], ins2[30:25], ins2[11:8], 1'b0};
    wire [12:0] br_imm3 = {ins3[31], ins3[7], ins3[30:25], ins3[11:8], 1'b0};

    wire [31:0] br_target0 = pc + {{19{br_imm0[12]}}, br_imm0};
    wire [31:0] br_target1 = pc + 4 + {{19{br_imm1[12]}}, br_imm1};
    wire [31:0] br_target2 = pc + 8 + {{19{br_imm2[12]}}, br_imm2};
    wire [31:0] br_target3 = pc + 12 + {{19{br_imm3[12]}}, br_imm3};

    // Compute JAL offsets (J-type)
    wire [20:0] jal_imm0 = {ins0[31], ins0[19:12], ins0[20], ins0[30:21], 1'b0};
    wire [20:0] jal_imm1 = {ins1[31], ins1[19:12], ins1[20], ins1[30:21], 1'b0};
    wire [20:0] jal_imm2 = {ins2[31], ins2[19:12], ins2[20], ins2[30:21], 1'b0};
    wire [20:0] jal_imm3 = {ins3[31], ins3[19:12], ins3[20], ins3[30:21], 1'b0};

    wire [31:0] jal_target0 = pc + {{11{jal_imm0[20]}}, jal_imm0};
    wire [31:0] jal_target1 = pc + 4 + {{11{jal_imm1[20]}}, jal_imm1};
    wire [31:0] jal_target2 = pc + 8 + {{11{jal_imm2[20]}}, jal_imm2};
    wire [31:0] jal_target3 = pc + 12 + {{11{jal_imm3[20]}}, jal_imm3};

    // Main combinational logic
    always @(*) begin


        // Scan instructions in order
        if (br0 || jal0) begin
            write_count = 1;
            next_pc = br0 ? br_target0 : jal_target0;
            inst_out0 = ins0 ;
            inst_out1 = 0;
            inst_out2 = 0;
            inst_out3 = 0;
        end
        else if (br1 || jal1) begin
            write_count = 2;
            next_pc = br1 ? br_target1 : jal_target1;
            inst_out0 = ins0 ;
            inst_out1 = ins1 ;
            inst_out2 =0;
            inst_out3 =0;
        end
        else if (br2 || jal2) begin
            write_count = 3;
            next_pc = br2 ? br_target2 : jal_target2;
            inst_out0 = ins0;
            inst_out1 = ins1;
            inst_out2 = ins2;
            inst_out3 = 0;
        end
        else if (br3 || jal3) begin
            write_count = 4;
            next_pc = br3 ? br_target3 : jal_target3;
            inst_out0 =ins0;
            inst_out1 =ins1;
            inst_out2 =ins2;
            inst_out3 =ins3;
        end

        else begin
        write_count = 4;
        next_pc = pc + 16; 
        inst_out0 =ins0;
        inst_out1 =ins1;
        inst_out2 =ins2;
        inst_out3 =ins3;
        end

    end

endmodule

`timescale 1ns/1ps

module tb_predecode;

    reg [31:0] pc;
    reg [31:0] ins0, ins1, ins2, ins3;
    wire [2:0] write_count;
    wire [31:0] next_pc;
    wire [31:0] inst_out0, inst_out1, inst_out2, inst_out3;

    // Instantiate predecode
    predecode uut(
        .pc(pc),
        .ins0(ins0),
        .ins1(ins1),
        .ins2(ins2),
        .ins3(ins3),
        .write_count(write_count),
        .next_pc(next_pc),
        .inst_out0(inst_out0),
        .inst_out1(inst_out1),
        .inst_out2(inst_out2),
        .inst_out3(inst_out3)
    );

    initial begin
        $dumpfile("predecode_tb.vcd");
        $dumpvars(0,tb_predecode);

        // Case 1: No branches in bundle
        pc = 0;
        ins0 = 32'h00000033; // ADD
        ins1 = 32'h00000013; // ADDI
        ins2 = 32'h00000013; // ADDI
        ins3 = 32'h00000033; // ADD
        #10;

        // Case 2: Branch at ins1
        pc = 16;
        ins0 = 32'h00000033; // ADD
        ins1 = 32'h00000063; // BEQ
        ins2 = 32'h00000013; // ADDI
        ins3 = 32'h00000033; // ADD
        #10;

        // Case 3: Branch at ins0
        pc = 32;
        ins0 = 32'h00000063; // BEQ
        ins1 = 32'h00000013; // ADDI
        ins2 = 32'h00000013; // ADDI
        ins3 = 32'h00000033; // ADD
        #10;

        // Case 4: Branch at ins3
        pc = 48;
        ins0 = 32'h00000033; // ADD
        ins1 = 32'h00000013; // ADDI
        ins2 = 32'h00000013; // ADDI
        ins3 = 32'h00000063; // BEQ
        #10;

        $finish;
    end

endmodule
