module Single_Cycle_Risc_V(master_reset,clk);
//interconnection of the modules of the datapath and interfacing with control path
input master_reset,clk;


//instantiation of the PC module
wire [63:0] current_pc, next_pc;
PC pc_unit(.pc_current(current_pc) , .pc_next(next_pc) , .reset(master_reset) , .clk(clk)); 

//instantiation of PC adder (PC <-- PC + 4)
wire [63:0] pc_inc_4;
pc_plus4_adder pc4adder( .pc_in(current_pc) , .pc_plus4(pc_inc_4));

//instantiation of PC branch target adder (PC <-- branch target address)

wire [63:0] imm_out; //imm_out is the output from immgen block (shift left done in immgen block itself)
wire [63:0] pc_branched;
pc_branch_adder pc_b_target(.pc_input(current_pc) , .pc_branch_target(pc_branched) , .offset_branch(imm_out));


jump_adder pc_jump_adder(.pc_inp(current_pc), .pc_jump_target(pc_jumped), .offset_jump(imm_out));

//instantiation of the branch control unit
wire branch , zero_alu , eff_sign_flag_alu , carryout_flag_alu , branch_tkn; //branch wire comes from the main control , zero_alu , eff_sign_flag_alu, carryout_flag_alu wire comes from the ALU
branch_control brch_cntrl(.branch_in_cntrl(branch) , .funct3_in(funct3), .zero_flag_in(zero_alu), .eff_sign_flag_in(eff_sign_flag_alu), .carryout_flag_in(carryout_flag_alu), .branch_taken_out(branch_tkn));

//instantiation of mux_3:1 (to select next PC value)
wire [63:0] pc_jumped; //this comes from pc_jump adder output
pcmux pc_select_mux( .pc_normal(pc_inc_4), .pc_branch_result(pc_branched), .pc_jump(pc_jumped), .next_pc_value(next_pc), .select_pc({jump_cntrl,branch_tkn}));


//instantiation of the instruction memory
wire [31:0] instruction_fetched ; 
instruction_memory inst_mem( .instn_out(instruction_fetched), .pc_input_address((current_pc[9:2]))) ; 

//instantiation of the register file

//extracting fields from instruction
wire [4:0] rs1, rs2, rd;
wire [6:0] opcode;
wire [2:0] funct3;
wire funct7;

assign opcode = instruction_fetched[6:0];
assign rd     = instruction_fetched[11:7];
assign funct3 = instruction_fetched[14:12];
assign rs1    = instruction_fetched[19:15];
assign rs2    = instruction_fetched[24:20];
assign funct7 = instruction_fetched[30]; // bit only needed for add/sub

wire write_into_reg_file; // this wire will come from control (to control when to write into the register)

wire [63:0] write_back_data; //this wire will come from data memory or ALU or pc+4 - controlled by mux (to provide data_in to be written into the reg file)

wire [63:0] register_data1 , register_data2; //for the read out values

register_file regfile( .read_reg_1(rs1) , .read_reg_2(rs2) , .write_reg(write_into_reg_file) , .write_reg_no(rd) , .data_in(write_back_data) , .clk(clk) , .read_out_1(register_data1) , .read_out_2(register_data2) , .reset(master_reset)) ;

//instantiation of the immgen unit
//both of the wires have been declared before :)
immgen immidiate_gen_unit( .instrn_input(instruction_fetched) , .sign_extended_out(imm_out));

//instantiation of mux2 (to select the 2nd ALU input)
wire [63:0] ALU_input_1;
assign ALU_input_1 = register_data1; 
wire [63:0] ALU_input_2;
wire ALU_src_control ; //this comes from the main control unit - indicates which should be ALU input 2 based on the OPcode
mux2_1 ALU_src_mux (.data_1(register_data2) , .data_2(imm_out) , .out_data(ALU_input_2) , .select(ALU_src_control));

//instantiation of the ALU
wire [3:0] ALU_CONTROL; //this will come from a seperate alu control unit
wire [63:0] ALU_OUT;

ALU alu_module( .alu_cntrl(ALU_CONTROL) , .data1(ALU_input_1) , .data2(ALU_input_2) , .alu_out(ALU_OUT) , .zero(zero_alu), .eff_sign(eff_sign_flag_alu), .carryout(carryout_flag_alu));

//instantiation of the ALU control
wire [1:0] ALU_OP; //this comes from main control unit 
alu_control alu_control_unit(.alu_op_in(ALU_OP) , .funct3_in(funct3) , .funct7_in(funct7) , .alu_control_out(ALU_CONTROL));

//instantiation of the data memory
wire MEM_READ , MEM_WRITE;
wire [63:0] data_memory_content;
// MEM_READ, MEM_WRITE comes from main control
// data_memory_content goes in a mux
data_memory datamem_unit(.input_address(ALU_OUT[10:3]) , .mem_read(MEM_READ), .mem_write(MEM_WRITE), .write_data_in(register_data2), .read_data_out(data_memory_content), .clk(clk)) ;

//instantiation of the write back mux

wire MEM_TO_REG;
data_write_mux mux_write_back_data_(.pc_4(pc_inc_4), .alu_result(ALU_OUT), .data_mem_info(data_memory_content), .data_write_out(write_back_data), .select_data({jump_cntrl,MEM_TO_REG}));
//mux2_1 write_back_mux (.data_1(ALU_OUT) , .data_2(data_memory_content) , .out_data(write_back_data) , .select(MEM_TO_REG)) ;

//instantiation of the main control unit
wire jump_cntrl;
main_control control_main(.opcode_in(opcode) , .alu_src(ALU_src_control) , .mem_to_reg(MEM_TO_REG), .reg_write_control(write_into_reg_file), . mem_read_control(MEM_READ), . mem_write_control(MEM_WRITE), .branch_control(branch), .alu_op_control(ALU_OP), .jal(jump_cntrl)) ; 


endmodule

//PC Module
module PC(pc_current,pc_next,reset,clk);
input [63:0] pc_next;
input reset,clk;
output reg [63:0] pc_current;

// PC register: updates to next PC on rising edge of clk, reset sets to 0

always @ (posedge clk) begin
    if(reset)
        pc_current <= 0;
    else
        pc_current <= pc_next;
end  
endmodule

//PC <-- PC + 4
module pc_plus4_adder (pc_in , pc_plus4);
input [63:0] pc_in;
output[63:0] pc_plus4;

assign pc_plus4 = pc_in + 64'd4; //incrementing the PC by 4

endmodule


//PC branch adder
module pc_branch_adder(pc_input , pc_branch_target , offset_branch);
input [63:0] pc_input , offset_branch;
output [63:0] pc_branch_target;

assign pc_branch_target = pc_input + offset_branch;

endmodule

module jump_adder(pc_inp , pc_jump_target , offset_jump);
input [63:0] pc_inp , offset_jump;
output [63:0] pc_jump_target;
assign pc_jump_target = pc_inp + offset_jump;
endmodule

//Mux 2:1
module mux2_1(data_1,data_2,out_data,select);
input [63:0] data_1 , data_2;
input select;
output [63:0] out_data;

assign out_data = select ? data_2 : data_1; // data_2 if select = 1  , data_1 if select = 0

endmodule

module pcmux(pc_normal , pc_branch_result, pc_jump , next_pc_value , select_pc);
input [63:0] pc_normal , pc_branch_result , pc_jump;
input [1:0] select_pc;
output reg [63:0] next_pc_value;
always @ (*) begin
    if(select_pc[1]) //select_pc[1] will have the jal control wire which indicates that its a j-type instruction
    next_pc_value = pc_jump;

    else if (select_pc == 2'b01)
    next_pc_value = pc_branch_result;

    else
    next_pc_value = pc_normal;
end
endmodule

module data_write_mux(pc_4 , alu_result , data_mem_info , data_write_out , select_data);
input [63:0] pc_4 , alu_result , data_mem_info;
input [1:0] select_data;
output reg [63:0] data_write_out;
always @ (*) begin
    if(select_data[1]) //select_data[1] will have the jal control signal wire
    data_write_out = pc_4;

    else if (select_data == 2'b01)
    data_write_out = data_mem_info;

    else 
    data_write_out = alu_result;

end
endmodule

//Register file
module register_file(read_reg_1 , read_reg_2 , write_reg , write_reg_no , data_in , clk, read_out_1 , read_out_2 , reset);
input clk,write_reg,reset;
input [4:0] read_reg_1,read_reg_2,write_reg_no;
output [63:0] read_out_1,read_out_2;
input [63:0] data_in;

reg [63:0] reg_array [31:0]; // 32 registers each of length 64 bits

//regs[register_number][bit_index] - 2D array of registers indexing

assign read_out_1 = (read_reg_1 == 0) ? 64'b0 : reg_array[read_reg_1]; 
assign read_out_2 = (read_reg_2 == 0) ? 64'b0 : reg_array[read_reg_2];

integer i;

always @ (posedge clk) begin
    if(reset) begin
        for(i = 0 ; i<64 ; i = i + 1)
        reg_array [i] <= 64'd0; 
    end
    else if(write_reg&(write_reg_no != 0)) begin
        reg_array[write_reg_no] <= data_in;
    end
end

endmodule

//Main control unit
module main_control(opcode_in , alu_src , mem_to_reg , reg_write_control , mem_read_control , mem_write_control , branch_control , alu_op_control , jal);
input [6:0] opcode_in;
output reg alu_src , mem_to_reg , reg_write_control , mem_read_control , mem_write_control , branch_control , jal;
output reg [1:0] alu_op_control;

always @ (*) begin
    case(opcode_in)
    7'b0110011 : /*R-Type*/ begin
        alu_src = 1'b0;
        mem_to_reg = 1'b0; 
        reg_write_control = 1'b1; 
        mem_read_control = 1'b0;
        mem_write_control = 1'b0;
        branch_control = 1'b0;
        alu_op_control = 2'b10;
        jal = 1'b0;
    end 
    7'b0000011 : /*I-Type (for load) */ begin
        alu_src = 1'b1;
        mem_to_reg = 1'b1; 
        reg_write_control = 1'b1; 
        mem_read_control = 1'b1;
        mem_write_control = 1'b0;
        branch_control = 1'b0;
        alu_op_control = 2'b00;
        jal = 1'b0;
    end
    7'b0100011 : /*S-Type*/ begin
        alu_src = 1'b1;
        mem_to_reg = 1'bx; 
        reg_write_control = 1'b0; 
        mem_read_control = 1'b0;
        mem_write_control = 1'b1;
        branch_control = 1'b0;
        alu_op_control = 2'b00;
        jal = 1'b0;
    end 
    7'b1100011 : /*SB-Type (for branch statements)*/begin
        alu_src = 1'b0;
        mem_to_reg = 1'bx; 
        reg_write_control = 1'b0; 
        mem_read_control = 1'b0;
        mem_write_control = 1'b0;
        branch_control = 1'b1;
        alu_op_control = 2'b01;
        jal = 1'b0;
    end

    7'b1101111 : /*for Jal type statements*/ begin
        alu_src = 1'bx;
        mem_to_reg = 1'b0; 
        reg_write_control = 1'b1; 
        mem_read_control = 1'b0;
        mem_write_control = 1'b0;
        branch_control = 1'b0;
        alu_op_control = 2'bxx;
        jal = 1'b1;
    end

    7'b0010011 : /*for addi type statements*/ begin
        alu_src = 1'b1;
        mem_to_reg = 1'b0; 
        reg_write_control = 1'b1; 
        mem_read_control = 1'b0;
        mem_write_control = 1'b0;
        branch_control = 1'b0;
        alu_op_control = 2'b10;
        jal = 1'b0;
    end

    default : begin
        alu_src = 1'b0;
        mem_to_reg = 1'b0; 
        reg_write_control = 1'b0; 
        mem_read_control = 1'b0;
        mem_write_control = 1'b0;
        branch_control = 1'b0;
        alu_op_control = 2'b00;
        jal = 1'b0;
    end
    endcase
end

endmodule


//Instruction memory
module instruction_memory(instn_out , pc_input_address);
input [7:0] pc_input_address;
output reg [31:0] instn_out;

reg [31:0] instn_mem [255:0]; //256 registers each of 32 bits --> can store 256 instructions 

//initializing the instruction memory with some instructions

initial begin
    instn_mem[0] = 32'b00000000100000000000001111101111; //jal x7, 8 //pc <-- 8 and pc = 4 is stored in reg x7
                   
    instn_mem[2] = 32'b00000000000000000011000010000011; // ld x1,0(x0) x1 = 10

    instn_mem[3] = 32'b00000000100000000011000100000011; // ld x2,8(x0) x2 = 3
    
    instn_mem[4] = 32'b00000000001000001000000110110011; // add x3,x1,x2 x3 = 13

    instn_mem[5] = 32'b01000000001000001000001000110011; // sub x4,x1,x2 x4 = 7

    instn_mem[6] = 32'b00000000001100000011110000100011; // sd x3,24(x0) , data_mem[3] = 13
    
    instn_mem[7] = 32'b00000001000000000011001010000011; // ld x5,16(x0) , x5 = 7

    instn_mem[8] = 32'b00000000010100100000100001100011; // beq x4,x5,16 , if true then pc = 32 + 16 = 48

    instn_mem[12] = 32'b00000000001100010001011001100011; // bne x2,x3,12 , if true then pc = 48 + 12 = 60

    instn_mem[15] = 32'b00000000001100010100100001100011; // blt x2,x3,16 , if true pc = 60 + 16 = 76

    instn_mem[19] = 32'b00000000000100010101010001100011; // bge x2,x1,8 , if true pc = 84 ,else pc = 80,84...

    instn_mem[20] = 32'b00000000000100111000010000110011; //add x8,x7,x1; x8 = 4 + 10 = 14
    
    instn_mem[21] = 32'b00000010100000000011000000100011; // sd x8 , 32(x0)

    instn_mem[22] = 32'b00000000010101000000010010010011; //addi x9, x8, 5 (alu_out = 19)

        end

//read instruction memory
always @ (*) begin
    instn_out = instn_mem[pc_input_address];
end
endmodule

//immidiate generation module
module immgen(instrn_input , sign_extended_out); //this module is used for sign extention of immidiate operands - for load,branch,store instructions
input [31:0] instrn_input;
output reg [63:0] sign_extended_out;
wire [6:0] opcode;
assign opcode = instrn_input[6:0];
always @ (*) begin
    case(opcode)
    7'b0110011 : sign_extended_out = 64'd0; // R-Type instructions do not need immgen module
    7'b0000011 : sign_extended_out = {{52{instrn_input[31]}} , instrn_input[31:20]};// I type (load)
    7'b0100011 : sign_extended_out = {{52{instrn_input[31]}} , instrn_input[31:25] , instrn_input[11:7]};// S type
    7'b1100011 : sign_extended_out = ( ({{52{instrn_input[31]}} , instrn_input[7] , instrn_input[30:25] , instrn_input[11:8]}) << 1 );// SB type (for beq instruction)
    7'b1101111 : sign_extended_out = (({{43{instrn_input[31]}} , instrn_input[19:12] , instrn_input[20] , instrn_input[30:21] , 1'b0})) ; // J - Type instructions
    7'b0010011 : sign_extended_out = {{52{instrn_input[31]}} , instrn_input[31:20]};
    default : sign_extended_out = 64'd0;
    endcase
end
endmodule

//Data Memory
module data_memory(input_address , mem_read , mem_write , write_data_in , read_data_out , clk);
input [7:0] input_address;
input mem_read , mem_write , clk;
input [63:0] write_data_in;
output reg [63:0] read_data_out;

//input address is from the ALU (byte indexing) - needs to be divided by 8 to ensure correct word indexing for data memory (done by  >> 3)

reg [63:0] data_mem [255:0] ; // 256 registers each of 64 bit length

integer i;  

initial begin
    // Preload data in data memory
    data_mem[0] = 64'd10;    // for ld x1, 0(x0)
    data_mem[1] = 64'd3;   // for ld x2, 8(x0)
    data_mem[2] = 64'd7;

    // rest of the data memory is 0
    for(i = 3; i < 256; i = i + 1)
        data_mem[i] = 64'd0;
end

always @ (posedge clk) begin
    if(mem_write)
        data_mem [input_address] <= write_data_in ;
end

always @ (*) begin
    if(mem_read)
        read_data_out = data_mem [input_address] ;
    else
        read_data_out = 64'd0;
end

endmodule

module branch_control(branch_in_cntrl, funct3_in, zero_flag_in , eff_sign_flag_in , carryout_flag_in ,branch_taken_out);
input [2:0] funct3_in;
input zero_flag_in , eff_sign_flag_in , carryout_flag_in ,branch_in_cntrl;
output reg branch_taken_out;
always @ (*) begin
    if (branch_in_cntrl) begin
    case(funct3_in)
    3'b000 : branch_taken_out = zero_flag_in ; //beq
    3'b001 : branch_taken_out = ~zero_flag_in ; //bne
    3'b100 : branch_taken_out = eff_sign_flag_in; //blt
    3'b101 : branch_taken_out = ~eff_sign_flag_in; //bge
    3'b110 : branch_taken_out = ~carryout_flag_in; //bltu
    3'b111 : branch_taken_out = carryout_flag_in; //bgeu
    default: branch_taken_out = 1'b0;
    endcase
    end

    else begin
        branch_taken_out = 0;
    end
end
endmodule


//ALU module
module ALU(alu_cntrl , data1 , data2 , alu_out , zero , eff_sign , carryout);
input [63:0] data1 , data2;
input [3:0] alu_cntrl;
output reg [63:0] alu_out;
output zero , eff_sign ,  carryout; // zero - used for beq,bne ; sign - used for signed bge,blt ; carryout - used for unsigned bge,blt
wire overflow , sign;
assign zero = (alu_out == 64'd0);
assign sign = alu_out[63];
assign overflow = (data1[63] != data2[63]) && (alu_out[63] != data1[63]);
assign eff_sign = overflow ^ sign; // used for signed comparision (sign  = 1 => blt , sign = 0 => bgt)
assign carryout = (data1 >= data2) ? 1 : 0; // used for unsigned comparision (blt if carryout = 0 , bgt if carryout = 1);

always @ (*) begin
    case(alu_cntrl)
    4'b0000: alu_out = data1 & data2; 
    4'b0001: alu_out = data1 | data2;
    4'b0010: alu_out = data1 + data2;
    4'b0110: alu_out = data1 - data2;
    default : alu_out = 64'd0;
    endcase
end

endmodule

//ALU Control
module alu_control(alu_op_in , funct3_in , funct7_in , alu_control_out);
input [1:0] alu_op_in;
input [2:0] funct3_in;
input funct7_in;
output reg [3:0] alu_control_out;

always @ (*) begin
    casex({alu_op_in,funct7_in,funct3_in}) // decoder using alu_op and the 2 funct fields
    6'b00xxxx : alu_control_out = 4'b0010; // for load and store instructions 
    6'b01xxxx : alu_control_out = 4'b0110; // for beq instructions
    6'b100000 : alu_control_out = 4'b0010; // for add instructions
    6'b101000 : alu_control_out = 4'b0110; // for sub instructions
    6'b100111 : alu_control_out = 4'b0000; // for and instructions
    6'b100110 : alu_control_out = 4'b0001;  // for or instructions
    default : alu_control_out = 4'b0000; 
    endcase
end

endmodule

// TESTBENCH
`timescale 1ns/1ps

`timescale 1ns/1ps

module tb_Single_Cycle_RISC_V;

    reg clk;
    reg master_reset;

    // Instantiate DUT
    Single_Cycle_Risc_V dut (
        .master_reset(master_reset),
        .clk(clk)
    );

    // Clock generation: 10 ns period
    always #5 clk = ~clk;

    initial begin
        // GTKWave setup
        $dumpfile("single_cycle_riscv.vcd");
        $dumpvars(0, tb_Single_Cycle_RISC_V);

        // Initialize signals
        clk = 0;
        master_reset = 1;

        // Hold reset for 1 cycle
        #10;
        master_reset = 0;

        // Run for 6 instructions = 6 cycles
        #200;

        
        $finish;
    end

endmodule

//this one is the complete implementation**** 