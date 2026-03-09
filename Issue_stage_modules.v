module IF_ID(clk,rst,ins0_out_F,ins1_out_F,ins2_out_F,ins3_out_F,write_count,ins0_out_D,ins1_out_D,ins2_out_D,ins3_out_D,valid_out_D);

input clk,rst;
input [31:0] ins0_out_F,ins1_out_F,ins2_out_F,ins3_out_F;
input [2:0] write_count; // comes directly from predecode
output reg [31:0] ins0_out_D,ins1_out_D,ins2_out_D,ins3_out_D;
output reg [3:0] valid_out_D;

// write_count=1 -> 4'b0001, =2 -> 4'b0011, =3 -> 4'b0111, =4 -> 4'b1111
reg [3:0] valid_in;
always@(*)begin
    case(write_count)
        3'd1: valid_in = 4'b0001;
        3'd2: valid_in = 4'b0011;
        3'd3: valid_in = 4'b0111;
        3'd4: valid_in = 4'b1111;
        default: valid_in = 4'b0000;
    endcase
end

always@(posedge clk or negedge rst)begin
    if(!rst) {ins0_out_D,ins1_out_D,ins2_out_D,ins3_out_D,valid_out_D} <= 132'd0;
    else begin
        ins0_out_D <= ins0_out_F;
        ins1_out_D <= ins1_out_F;
        ins2_out_D <= ins2_out_F;
        ins3_out_D <= ins3_out_F;
        valid_out_D <= valid_in;
    end
end

endmodule



module Decode(ins_0,ins_1,ins_2,ins_3,valid_in,opcode_0,opcode_1,opcode_2,opcode_3,rs1_0,rs1_1,rs1_2,rs1_3,rs2_0,rs2_1,rs2_2,rs2_3,rd_0,rd_1,rd_2,rd_3,imm_0,imm_1,imm_2,imm_3,func7_0,func7_1,func7_2,func7_3,func3_0,func3_1,func3_2,func3_3,has_dest,is_branch,is_jump,is_jalr,is_load,is_store,valid_out);

input [31:0] ins_0,ins_1,ins_2,ins_3;
input [3:0] valid_in;
// valid_in is a thermometer code from predecode output
// write_count=1 -> valid_in=4'b0001
// write_count=2 -> valid_in=4'b0011
// write_count=3 -> valid_in=4'b0111
// write_count=4 -> valid_in=4'b1111

output reg [6:0] opcode_0,opcode_1,opcode_2,opcode_3;
output reg [4:0] rs1_0,rs1_1,rs1_2,rs1_3;
output reg [4:0] rs2_0,rs2_1,rs2_2,rs2_3;
output reg [4:0] rd_0,rd_1,rd_2,rd_3;
output reg [31:0] imm_0,imm_1,imm_2,imm_3;
output reg [6:0] func7_0,func7_1,func7_2,func7_3;
output reg [2:0] func3_0,func3_1,func3_2,func3_3;

output reg [3:0] has_dest,is_branch,is_jump,is_jalr,is_load,is_store;
output reg [3:0] valid_out;

always@(*)begin

    opcode_0 = ins_0[6:0];
    opcode_1 = ins_1[6:0];
    opcode_2 = ins_2[6:0];
    opcode_3 = ins_3[6:0];
    valid_out = valid_in;

    {rs1_0,rs1_1,rs1_2,rs1_3,rs2_0,rs2_1,rs2_2,rs2_3,rd_0,rd_1,rd_2,rd_3,imm_0,imm_1,imm_2,imm_3,func7_0,func7_1,func7_2,func7_3,func3_0,func3_1,func3_2,func3_3,has_dest,is_branch,is_jump,is_jalr,is_load,is_store} = 252'd0;

    if(valid_in[0])begin
        case(opcode_0)

        7'b0110011: begin //R-type
                    rs1_0 = ins_0[19:15];
                    rs2_0 = ins_0[24:20];
                    rd_0 = ins_0[11:7];
                    func7_0 = ins_0[31:25];
                    func3_0 = ins_0[14:12];
                    has_dest[0] = 1'b1;
                    end

        7'b0010011: begin //I-type
                    rs1_0 = ins_0[19:15];
                    rd_0 = ins_0[11:7];
                    imm_0 = {{20{ins_0[31]}},ins_0[31:20]};
                    func3_0 = ins_0[14:12];
                    has_dest[0] = 1'b1;
                    end

        7'b0000011: begin //Load
                    rs1_0 = ins_0[19:15];
                    rd_0 = ins_0[11:7];
                    imm_0 = {{20{ins_0[31]}},ins_0[31:20]};
                    func3_0 = ins_0[14:12];
                    has_dest[0] = 1'b1;
                    is_load[0] = 1'b1;
                    end

        7'b0100011: begin //S-type
                    rs1_0 = ins_0[19:15];
                    rs2_0 = ins_0[24:20];
                    imm_0 = {{20{ins_0[31]}},ins_0[31:25],ins_0[11:7]};
                    func3_0 = ins_0[14:12];
                    is_store[0] = 1'b1;
                    end

        7'b1100011: begin //SB-type
                    rs1_0 = ins_0[19:15];
                    rs2_0 = ins_0[24:20];
                    imm_0 = {{19{ins_0[31]}},ins_0[31],ins_0[7],ins_0[30:25],ins_0[11:8],1'b0};
                    func3_0 = ins_0[14:12];
                    is_branch[0] = 1'b1;
                    end

        7'b1101111: begin //JAL
                    rd_0 = ins_0[11:7];
                    imm_0 = {{11{ins_0[31]}},ins_0[31],ins_0[19:12],ins_0[20],ins_0[30:21],1'b0};
                    has_dest[0] = 1'b1;
                    is_jump[0] = 1'b1;
                    end

        7'b1100111: begin //JALR
                    rs1_0 = ins_0[19:15];
                    rd_0 = ins_0[11:7];
                    imm_0 = {{20{ins_0[31]}},ins_0[31:20]};
                    func3_0 = ins_0[14:12];
                    has_dest[0] = 1'b1;
                    is_jalr[0] = 1'b1;
                    end

        endcase
    end

    if(valid_in[1])begin
        case(opcode_1)

        7'b0110011: begin //R-type
                    rs1_1 = ins_1[19:15];
                    rs2_1 = ins_1[24:20];
                    rd_1 = ins_1[11:7];
                    func7_1 = ins_1[31:25];
                    func3_1 = ins_1[14:12];
                    has_dest[1] = 1'b1;
                    end

        7'b0010011: begin //I-type
                    rs1_1 = ins_1[19:15];
                    rd_1 = ins_1[11:7];
                    imm_1 = {{20{ins_1[31]}},ins_1[31:20]};
                    func3_1 = ins_1[14:12];
                    has_dest[1] = 1'b1;
                    end

        7'b0000011: begin //Load
                    rs1_1 = ins_1[19:15];
                    rd_1 = ins_1[11:7];
                    imm_1 = {{20{ins_1[31]}},ins_1[31:20]};
                    func3_1 = ins_1[14:12];
                    has_dest[1] = 1'b1;
                    is_load[1] = 1'b1;
                    end

        7'b0100011: begin //S-type
                    rs1_1 = ins_1[19:15];
                    rs2_1 = ins_1[24:20];
                    imm_1 = {{20{ins_1[31]}},ins_1[31:25],ins_1[11:7]};
                    func3_1 = ins_1[14:12];
                    is_store[1] = 1'b1;
                    end

        7'b1100011: begin //SB-type
                    rs1_1 = ins_1[19:15];
                    rs2_1 = ins_1[24:20];
                    imm_1 = {{19{ins_1[31]}},ins_1[31],ins_1[7],ins_1[30:25],ins_1[11:8],1'b0};
                    func3_1 = ins_1[14:12];
                    is_branch[1] = 1'b1;
                    end

        7'b1101111: begin //JAL
                    rd_1 = ins_1[11:7];
                    imm_1 = {{11{ins_1[31]}},ins_1[31],ins_1[19:12],ins_1[20],ins_1[30:21],1'b0};
                    has_dest[1] = 1'b1;
                    is_jump[1] = 1'b1;
                    end

        7'b1100111: begin //JALR
                    rs1_1 = ins_1[19:15];
                    rd_1 = ins_1[11:7];
                    imm_1 = {{20{ins_1[31]}},ins_1[31:20]};
                    func3_1 = ins_1[14:12];
                    has_dest[1] = 1'b1;
                    is_jalr[1] = 1'b1;
                    end

        endcase
    end

    if(valid_in[2])begin
        case(opcode_2)

        7'b0110011: begin //R-type
                    rs1_2 = ins_2[19:15];
                    rs2_2 = ins_2[24:20];
                    rd_2 = ins_2[11:7];
                    func7_2 = ins_2[31:25];
                    func3_2 = ins_2[14:12];
                    has_dest[2] = 1'b1;
                    end

        7'b0010011: begin //I-type
                    rs1_2 = ins_2[19:15];
                    rd_2 = ins_2[11:7];
                    imm_2 = {{20{ins_2[31]}},ins_2[31:20]};
                    func3_2 = ins_2[14:12];
                    has_dest[2] = 1'b1;
                    end

        7'b0000011: begin //Load
                    rs1_2 = ins_2[19:15];
                    rd_2 = ins_2[11:7];
                    imm_2 = {{20{ins_2[31]}},ins_2[31:20]};
                    func3_2 = ins_2[14:12];
                    has_dest[2] = 1'b1;
                    is_load[2] = 1'b1;
                    end

        7'b0100011: begin //S-type
                    rs1_2 = ins_2[19:15];
                    rs2_2 = ins_2[24:20];
                    imm_2 = {{20{ins_2[31]}},ins_2[31:25],ins_2[11:7]};
                    func3_2 = ins_2[14:12];
                    is_store[2] = 1'b1;
                    end

        7'b1100011: begin //SB-type
                    rs1_2 = ins_2[19:15];
                    rs2_2 = ins_2[24:20];
                    imm_2 = {{19{ins_2[31]}},ins_2[31],ins_2[7],ins_2[30:25],ins_2[11:8],1'b0};
                    func3_2 = ins_2[14:12];
                    is_branch[2] = 1'b1;
                    end

        7'b1101111: begin //JAL
                    rd_2 = ins_2[11:7];
                    imm_2 = {{11{ins_2[31]}},ins_2[31],ins_2[19:12],ins_2[20],ins_2[30:21],1'b0};
                    has_dest[2] = 1'b1;
                    is_jump[2] = 1'b1;
                    end

        7'b1100111: begin //JALR
                    rs1_2 = ins_2[19:15];
                    rd_2 = ins_2[11:7];
                    imm_2 = {{20{ins_2[31]}},ins_2[31:20]};
                    func3_2 = ins_2[14:12];
                    has_dest[2] = 1'b1;
                    is_jalr[2] = 1'b1;
                    end

        endcase
    end

    if(valid_in[3])begin
        case(opcode_3)

        7'b0110011: begin //R-type
                    rs1_3 = ins_3[19:15];
                    rs2_3 = ins_3[24:20];
                    rd_3 = ins_3[11:7];
                    func7_3 = ins_3[31:25];
                    func3_3 = ins_3[14:12];
                    has_dest[3] = 1'b1;
                    end

        7'b0010011: begin //I-type
                    rs1_3 = ins_3[19:15];
                    rd_3 = ins_3[11:7];
                    imm_3 = {{20{ins_3[31]}},ins_3[31:20]};
                    func3_3 = ins_3[14:12];
                    has_dest[3] = 1'b1;
                    end

        7'b0000011: begin //Load
                    rs1_3 = ins_3[19:15];
                    rd_3 = ins_3[11:7];
                    imm_3 = {{20{ins_3[31]}},ins_3[31:20]};
                    func3_3 = ins_3[14:12];
                    has_dest[3] = 1'b1;
                    is_load[3] = 1'b1;
                    end

        7'b0100011: begin //S-type
                    rs1_3 = ins_3[19:15];
                    rs2_3 = ins_3[24:20];
                    imm_3 = {{20{ins_3[31]}},ins_3[31:25],ins_3[11:7]};
                    func3_3 = ins_3[14:12];
                    is_store[3] = 1'b1;
                    end

        7'b1100011: begin //SB-type
                    rs1_3 = ins_3[19:15];
                    rs2_3 = ins_3[24:20];
                    imm_3 = {{19{ins_3[31]}},ins_3[31],ins_3[7],ins_3[30:25],ins_3[11:8],1'b0};
                    func3_3 = ins_3[14:12];
                    is_branch[3] = 1'b1;
                    end

        7'b1101111: begin //JAL
                    rd_3 = ins_3[11:7];
                    imm_3 = {{11{ins_3[31]}},ins_3[31],ins_3[19:12],ins_3[20],ins_3[30:21],1'b0};
                    has_dest[3] = 1'b1;
                    is_jump[3] = 1'b1;
                    end

        7'b1100111: begin //JALR
                    rs1_3 = ins_3[19:15];
                    rd_3 = ins_3[11:7];
                    imm_3 = {{20{ins_3[31]}},ins_3[31:20]};
                    func3_3 = ins_3[14:12];
                    has_dest[3] = 1'b1;
                    is_jalr[3] = 1'b1;
                    end

        endcase
    end

end

endmodule



module RAT(clk,rst,rs1_0,rs1_1,rs1_2,rs1_3,rs2_0,rs2_1,rs2_2,rs2_3,rd_0,rd_1,rd_2,rd_3,update_valid,new_rd_0,new_rd_1,new_rd_2,new_rd_3,new_prd_0,new_prd_1,new_prd_2,new_prd_3,prs1_0,prs1_1,prs1_2,prs1_3,prs2_0,prs2_1,prs2_2,prs2_3,old_prd_0,old_prd_1,old_prd_2,old_prd_3);

input clk,rst;
input [4:0] rs1_0,rs1_1,rs1_2,rs1_3;
input [4:0] rs2_0,rs2_1,rs2_2,rs2_3;
input [4:0] rd_0,rd_1,rd_2,rd_3;
input [3:0] update_valid;
input [4:0] new_rd_0,new_rd_1,new_rd_2,new_rd_3;
input [6:0] new_prd_0,new_prd_1,new_prd_2,new_prd_3;

output reg [6:0] prs1_0,prs1_1,prs1_2,prs1_3;
output reg [6:0] prs2_0,prs2_1,prs2_2,prs2_3;
output reg [6:0] old_prd_0,old_prd_1,old_prd_2,old_prd_3;

reg [6:0] RAT_main [0:31];
reg [6:0] RAT_temp [0:31];
integer i;

always@(*)begin

    for(i = 0; i < 32; i = i + 1)
        RAT_temp[i] = RAT_main[i];

    //INSTRUCTION 0
    prs1_0    = (update_valid[0] && rs1_0!=5'd0) ? RAT_temp[rs1_0] : 7'd0;
    prs2_0    = (update_valid[0] && rs2_0!=5'd0) ? RAT_temp[rs2_0] : 7'd0;
    old_prd_0 = (update_valid[0] &&  rd_0!=5'd0) ? RAT_temp[rd_0]  : 7'd0;
    if(update_valid[0] && new_rd_0!=5'd0)
        RAT_temp[new_rd_0] = new_prd_0;

    //INSTRUCTION 1
    prs1_1    = (update_valid[1] && rs1_1!=5'd0) ? RAT_temp[rs1_1] : 7'd0;
    prs2_1    = (update_valid[1] && rs2_1!=5'd0) ? RAT_temp[rs2_1] : 7'd0;
    old_prd_1 = (update_valid[1] &&  rd_1!=5'd0) ? RAT_temp[rd_1]  : 7'd0;
    if(update_valid[1] && new_rd_1!=5'd0)
        RAT_temp[new_rd_1] = new_prd_1;

    //INSTRUCTION 2
    prs1_2    = (update_valid[2] && rs1_2!=5'd0) ? RAT_temp[rs1_2] : 7'd0;
    prs2_2    = (update_valid[2] && rs2_2!=5'd0) ? RAT_temp[rs2_2] : 7'd0;
    old_prd_2 = (update_valid[2] &&  rd_2!=5'd0) ? RAT_temp[rd_2]  : 7'd0;
    if(update_valid[2] && new_rd_2!=5'd0)
        RAT_temp[new_rd_2] = new_prd_2;

    //INSTRUCTION 3
    prs1_3    = (update_valid[3] && rs1_3!=5'd0) ? RAT_temp[rs1_3] : 7'd0;
    prs2_3    = (update_valid[3] && rs2_3!=5'd0) ? RAT_temp[rs2_3] : 7'd0;
    old_prd_3 = (update_valid[3] &&  rd_3!=5'd0) ? RAT_temp[rd_3]  : 7'd0;
    if(update_valid[3] && new_rd_3!=5'd0)
        RAT_temp[new_rd_3] = new_prd_3;

end

always@(posedge clk, negedge rst)begin
    if(!rst)begin
        for(i = 0; i < 32; i = i + 1)
            RAT_main[i] <= i[6:0];
    end
    else begin
        if(update_valid[0] && new_rd_0!=5'd0) RAT_main[new_rd_0] <= new_prd_0;
        if(update_valid[1] && new_rd_1!=5'd0) RAT_main[new_rd_1] <= new_prd_1;
        if(update_valid[2] && new_rd_2!=5'd0) RAT_main[new_rd_2] <= new_prd_2;
        if(update_valid[3] && new_rd_3!=5'd0) RAT_main[new_rd_3] <= new_prd_3;
    end
end

endmodule

module Rename_Dispatch(clk,rst,
                       rs1_0,rs1_1,rs1_2,rs1_3,
                       rs2_0,rs2_1,rs2_2,rs2_3,
                       rd_0,rd_1,rd_2,rd_3,
                       imm_0,imm_1,imm_2,imm_3,
                       func7_0,func7_1,func7_2,func7_3,
                       func3_0,func3_1,func3_2,func3_3,
                       opcode_0,opcode_1,opcode_2,opcode_3,
                       has_dest,is_branch,is_jump,is_jalr,is_load,is_store,
                       valid_in,
                       rat_rs1_0,rat_rs1_1,rat_rs1_2,rat_rs1_3,
                       rat_rs2_0,rat_rs2_1,rat_rs2_2,rat_rs2_3,
                       rat_rd_0,rat_rd_1,rat_rd_2,rat_rd_3,
                       rat_prs1_0,rat_prs1_1,rat_prs1_2,rat_prs1_3,
                       rat_prs2_0,rat_prs2_1,rat_prs2_2,rat_prs2_3,
                       rat_old_prd_0,rat_old_prd_1,rat_old_prd_2,rat_old_prd_3,
                       rat_update_valid,
                       rat_new_rd_0,rat_new_rd_1,rat_new_rd_2,rat_new_rd_3,
                       rat_new_prd_0,rat_new_prd_1,rat_new_prd_2,rat_new_prd_3,
                       fl_alloc_req,
                       fl_alloc_preg_0,fl_alloc_preg_1,fl_alloc_preg_2,fl_alloc_preg_3,
                       fl_stall,
                       prs1_0,prs1_1,prs1_2,prs1_3,
                       prs2_0,prs2_1,prs2_2,prs2_3,
                       prd_0,prd_1,prd_2,prd_3,
                       old_prd_0,old_prd_1,old_prd_2,old_prd_3,
                       imm_out_0,imm_out_1,imm_out_2,imm_out_3,
                       func7_out_0,func7_out_1,func7_out_2,func7_out_3,
                       func3_out_0,func3_out_1,func3_out_2,func3_out_3,
                       opcode_out_0,opcode_out_1,opcode_out_2,opcode_out_3,
                       has_dest_out,is_branch_out,is_jump_out,is_jalr_out,is_load_out,is_store_out,
                       valid_out,
                       stall);

input clk,rst;

// from Decode
input [4:0] rs1_0,rs1_1,rs1_2,rs1_3;
input [4:0] rs2_0,rs2_1,rs2_2,rs2_3;
input [4:0] rd_0,rd_1,rd_2,rd_3;
input [31:0] imm_0,imm_1,imm_2,imm_3;
input [6:0] func7_0,func7_1,func7_2,func7_3;
input [2:0] func3_0,func3_1,func3_2,func3_3;
input [6:0] opcode_0,opcode_1,opcode_2,opcode_3;
input [3:0] has_dest,is_branch,is_jump,is_jalr,is_load,is_store;
input [3:0] valid_in;

// RAT interface
output [4:0] rat_rs1_0,rat_rs1_1,rat_rs1_2,rat_rs1_3;
output [4:0] rat_rs2_0,rat_rs2_1,rat_rs2_2,rat_rs2_3;
output [4:0] rat_rd_0,rat_rd_1,rat_rd_2,rat_rd_3;
input  [6:0] rat_prs1_0,rat_prs1_1,rat_prs1_2,rat_prs1_3;
input  [6:0] rat_prs2_0,rat_prs2_1,rat_prs2_2,rat_prs2_3;
input  [6:0] rat_old_prd_0,rat_old_prd_1,rat_old_prd_2,rat_old_prd_3;
output reg [3:0] rat_update_valid;
output [4:0] rat_new_rd_0,rat_new_rd_1,rat_new_rd_2,rat_new_rd_3;
output [6:0] rat_new_prd_0,rat_new_prd_1,rat_new_prd_2,rat_new_prd_3;

// Free List interface
output [3:0] fl_alloc_req;
input  [6:0] fl_alloc_preg_0,fl_alloc_preg_1,fl_alloc_preg_2,fl_alloc_preg_3;
input  fl_stall;

// outputs to Issue Queue
output reg [6:0] prs1_0,prs1_1,prs1_2,prs1_3;
output reg [6:0] prs2_0,prs2_1,prs2_2,prs2_3;
output reg [6:0] prd_0,prd_1,prd_2,prd_3;
output reg [6:0] old_prd_0,old_prd_1,old_prd_2,old_prd_3;
output reg [31:0] imm_out_0,imm_out_1,imm_out_2,imm_out_3;
output reg [6:0] func7_out_0,func7_out_1,func7_out_2,func7_out_3;
output reg [2:0] func3_out_0,func3_out_1,func3_out_2,func3_out_3;
output reg [6:0] opcode_out_0,opcode_out_1,opcode_out_2,opcode_out_3;
output reg [3:0] has_dest_out,is_branch_out,is_jump_out,is_jalr_out,is_load_out,is_store_out;
output reg [3:0] valid_out;
output reg stall;

// wire decode rs1,rs2,rd straight to RAT lookup ports
assign rat_rs1_0 = rs1_0;
assign rat_rs1_1 = rs1_1;
assign rat_rs1_2 = rs1_2;
assign rat_rs1_3 = rs1_3;

assign rat_rs2_0 = rs2_0;
assign rat_rs2_1 = rs2_1;
assign rat_rs2_2 = rs2_2;
assign rat_rs2_3 = rs2_3;

assign rat_rd_0 = rd_0;
assign rat_rd_1 = rd_1;
assign rat_rd_2 = rd_2;
assign rat_rd_3 = rd_3;

// wire rd and newly allocated physical registers to RAT update ports
assign rat_new_rd_0 = rd_0;
assign rat_new_rd_1 = rd_1;
assign rat_new_rd_2 = rd_2;
assign rat_new_rd_3 = rd_3;

assign rat_new_prd_0 = fl_alloc_preg_0;
assign rat_new_prd_1 = fl_alloc_preg_1;
assign rat_new_prd_2 = fl_alloc_preg_2;
assign rat_new_prd_3 = fl_alloc_preg_3;

// request a free physical register only for valid instructions that write to a non-zero rd
assign fl_alloc_req[0] = valid_in[0] && has_dest[0] && (rd_0 != 5'd0);
assign fl_alloc_req[1] = valid_in[1] && has_dest[1] && (rd_1 != 5'd0);
assign fl_alloc_req[2] = valid_in[2] && has_dest[2] && (rd_2 != 5'd0);
assign fl_alloc_req[3] = valid_in[3] && has_dest[3] && (rd_3 != 5'd0);

always@(*)begin
    stall           = fl_stall;
    rat_update_valid = fl_alloc_req;
end

always@(posedge clk, negedge rst)begin

    if(!rst)begin
        {prs1_0,prs1_1,prs1_2,prs1_3}                                      <= 28'd0;
        {prs2_0,prs2_1,prs2_2,prs2_3}                                      <= 28'd0;
        {prd_0,prd_1,prd_2,prd_3}                                          <= 28'd0;
        {old_prd_0,old_prd_1,old_prd_2,old_prd_3}                          <= 28'd0;
        {imm_out_0,imm_out_1,imm_out_2,imm_out_3}                          <= 128'd0;
        {func7_out_0,func7_out_1,func7_out_2,func7_out_3}                  <= 28'd0;
        {func3_out_0,func3_out_1,func3_out_2,func3_out_3}                  <= 12'd0;
        {opcode_out_0,opcode_out_1,opcode_out_2,opcode_out_3}              <= 28'd0;
        {has_dest_out,is_branch_out,is_jump_out,is_jalr_out,is_load_out,is_store_out} <= 24'd0;
        valid_out <= 4'd0;
    end

    else if(!stall)begin
        prs1_0 <= rat_prs1_0;   prs1_1 <= rat_prs1_1;   prs1_2 <= rat_prs1_2;   prs1_3 <= rat_prs1_3;
        prs2_0 <= rat_prs2_0;   prs2_1 <= rat_prs2_1;   prs2_2 <= rat_prs2_2;   prs2_3 <= rat_prs2_3;

        prd_0 <= fl_alloc_preg_0;
        prd_1 <= fl_alloc_preg_1;
        prd_2 <= fl_alloc_preg_2;
        prd_3 <= fl_alloc_preg_3;

        old_prd_0 <= rat_old_prd_0; old_prd_1 <= rat_old_prd_1;
        old_prd_2 <= rat_old_prd_2; old_prd_3 <= rat_old_prd_3;

        imm_out_0 <= imm_0;     imm_out_1 <= imm_1;     imm_out_2 <= imm_2;     imm_out_3 <= imm_3;
        func7_out_0 <= func7_0; func7_out_1 <= func7_1; func7_out_2 <= func7_2; func7_out_3 <= func7_3;
        func3_out_0 <= func3_0; func3_out_1 <= func3_1; func3_out_2 <= func3_2; func3_out_3 <= func3_3;
        opcode_out_0 <= opcode_0; opcode_out_1 <= opcode_1; opcode_out_2 <= opcode_2; opcode_out_3 <= opcode_3;

        has_dest_out  <= has_dest;
        is_branch_out <= is_branch;
        is_jump_out   <= is_jump;
        is_jalr_out   <= is_jalr;
        is_load_out   <= is_load;
        is_store_out  <= is_store;
        valid_out     <= valid_in;
    end

end

endmodule

module Free_List(clk,rst,alloc_req,alloc_preg_0,alloc_preg_1,alloc_preg_2,alloc_preg_3,stall,rob_free_valid,rob_free_preg);

input clk,rst;
input [3:0] alloc_req;
output reg [6:0] alloc_preg_0,alloc_preg_1,alloc_preg_2,alloc_preg_3;
output reg stall;
input rob_free_valid;
input [6:0] rob_free_preg;

reg [6:0] free_list [0:95];
reg [6:0] fl_head;
reg [6:0] fl_tail;
reg [6:0] fl_count;

wire [2:0] total_req;
assign total_req = alloc_req[0] + alloc_req[1] + alloc_req[2] + alloc_req[3];

integer i;
integer offset;

always@(*)begin
    stall = (fl_count < total_req);
    offset = 0;
    alloc_preg_0 = 7'd0;
    alloc_preg_1 = 7'd0;
    alloc_preg_2 = 7'd0;
    alloc_preg_3 = 7'd0;

    if(alloc_req[0])begin
        alloc_preg_0 = free_list[(fl_head + offset) % 96];
        offset = offset + 1;
    end
    if(alloc_req[1])begin
        alloc_preg_1 = free_list[(fl_head + offset) % 96];
        offset = offset + 1;
    end
    if(alloc_req[2])begin
        alloc_preg_2 = free_list[(fl_head + offset) % 96];
        offset = offset + 1;
    end
    if(alloc_req[3])begin
        alloc_preg_3 = free_list[(fl_head + offset) % 96];
    end
end

always@(posedge clk, negedge rst)begin
    if(!rst)begin
        for(i = 0; i < 96; i = i + 1)
            free_list[i] <= (7'b0100000 + i[6:0]); // p32 to p127
        fl_head  <= 7'd0;
        fl_tail  <= 7'd0;
        fl_count <= 7'd96;
    end
    else begin
        // advance head by how many were allocated this cycle
        fl_head <= (!stall) ? ((fl_head + total_req) % 96) : fl_head;

        // advance tail and write returned register when ROB frees one
        if(rob_free_valid && rob_free_preg != 7'd0 && rob_free_preg >= 32)begin
            free_list[fl_tail] <= rob_free_preg;
            fl_tail <= (fl_tail + 7'd1) % 96;
        end

        // count = count - allocated + returned
        fl_count <= fl_count
                    - ((!stall) ? total_req : 3'd0)
                    + ((rob_free_valid && rob_free_preg != 7'd0 && rob_free_preg >= 32) ? 7'd1 : 7'd0);
    end
end

endmodule