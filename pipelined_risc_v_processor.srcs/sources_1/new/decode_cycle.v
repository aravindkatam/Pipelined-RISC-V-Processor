`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/23/2025 07:07:11 AM
// Design Name: 
// Module Name: decode_cycle
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module alu_decoder(
    input [1:0] aluop,
    input [2:0] funct3,
    input [6:0] funct7, op,
    output [2:0] alucontrol
);

    reg [2:0] alucontrol_r;
    assign alucontrol = alucontrol_r;

    always @(*) begin
        case (aluop)
            2'b00: alucontrol_r = 3'b000; // Load/Store = ADD
            2'b01: alucontrol_r = 3'b001; // Branch = SUB
            2'b10: begin
                case (funct3)
                    3'b000: alucontrol_r = (funct7[5] == 1 && op[5] == 1) ? 3'b001 : 3'b000; // ADD/SUB
                    3'b010: alucontrol_r = 3'b101; // SLT
                    3'b110: alucontrol_r = 3'b011; // OR
                    3'b111: alucontrol_r = 3'b010; // AND
                    default: alucontrol_r = 3'b000;
                endcase
            end
            default: alucontrol_r = 3'b000;
        endcase
    end
endmodule


//====================== Main Decoder =====================
module main_decoder(
    input [6:0] op,
    output reg regwrite, alusrc, memwrite, branch,
    output reg [1:0] immsrc, resultsrc,
    output reg [1:0] aluop
);

always @(*) begin
    case (op)
        7'b0000011: begin // Load (LW)
            regwrite = 1; alusrc = 1; memwrite = 0;
            resultsrc = 2'b01; branch = 0; aluop = 2'b00; immsrc = 2'b00;
        end
        7'b0100011: begin // Store (SW)
            regwrite = 0; alusrc = 1; memwrite = 1;
            resultsrc = 2'b00; branch = 0; aluop = 2'b00; immsrc = 2'b01;
        end
        7'b1100011: begin // Branch (BEQ)
            regwrite = 0; alusrc = 0; memwrite = 0;
            resultsrc = 2'b00; branch = 1; aluop = 2'b01; immsrc = 2'b10;
        end
        7'b0110011: begin // R-type (ADD/SUB)
            regwrite = 1; alusrc = 0; memwrite = 0;
            resultsrc = 2'b00; branch = 0; aluop = 2'b10; immsrc = 2'b00;
        end
        7'b0010011: begin // I-type ALU (ADDI, SLTI, etc.)
            regwrite = 1; alusrc = 1; memwrite = 0;
            resultsrc = 2'b00; branch = 0; aluop = 2'b00; immsrc = 2'b00;
        end
        default: begin
            regwrite = 0; alusrc = 0; memwrite = 0;
            resultsrc = 2'b00; branch = 0; aluop = 2'b00; immsrc = 2'b00;
        end
    endcase
end
endmodule


//====================== Control Unit Top =====================
module control_unit_top(
    input [6:0] op, funct7,
    input [2:0] funct3,
    output regwrite, alusrc, memwrite, branch,
    output [1:0] immsrc, resultsrce,
    output [2:0] alucontrol
);

    wire [1:0] aluop;
    wire [1:0] resultsrc_wire;

    main_decoder md (
        .op(op),
        .regwrite(regwrite),
        .alusrc(alusrc),
        .memwrite(memwrite),
        .resultsrc(resultsrc_wire),
        .branch(branch),
        .aluop(aluop),
        .immsrc(immsrc)
    );

    alu_decoder ad (
        .aluop(aluop),
        .funct3(funct3),
        .funct7(funct7),
        .op(op),
        .alucontrol(alucontrol)
    );

    assign resultsrce = resultsrc_wire; // ✅ fix

endmodule

//====================== Register File =====================
module register_file(clk, rst, we3, wd3, a1, a2, a3, rd1, rd2);
    input clk, rst, we3;
    input [4:0] a1, a2, a3;
    input [31:0] wd3;
    output [31:0] rd1, rd2;

    reg [31:0] register[0:31];

    always @(posedge clk)
        if (we3 && a3 != 5'd0)
            register[a3] <= wd3;

    assign rd1 = (rst == 0) ? 32'd0 : register[a1];
    assign rd2 = (rst == 0) ? 32'd0 : register[a2];

    initial begin
        register[0] = 32'd0;   register[1] = 32'd5;
        register[2] = 32'd10;  register[3] = 32'd15;
        register[4] = 32'd100; register[5] = 32'd200;
        register[6] = 32'd1;   register[7] = 32'd2;
        register[8] = 32'd4;   register[9] = 32'd8;
        register[10] = 32'hFF; register[11] = 32'hFFF;
        register[12] = 32'hFFFF;
    end
endmodule

//====================== Sign Extend =====================
module sign_extend(in, immsrc, imm_ext);
    input [31:0] in;
    input [1:0] immsrc;
    output [31:0] imm_ext;

    assign imm_ext = (immsrc == 2'b00) ? {{20{in[31]}}, in[31:20]} :
                     (immsrc == 2'b01) ? {{20{in[31]}}, in[31:25], in[11:7]} :
                     (immsrc == 2'b10) ? {{20{in[31]}}, in[7], in[30:25], in[11:8], 1'b0} :
                     32'b0;
endmodule

//====================== Decode Cycle =====================
module decode_cycle(
    input clk, rst, regwritew,
    input [4:0] rd_w,
    input [31:0] instrd, pcd, pcplus4d, resultw,
    output regwritee, alusrce, memwritee, branche,
    output [1:0] resultsrce, immsrce,
    output [2:0] alucontrole,
    output [31:0] rd1_e, rd2_e, imm_exte,
    output [31:0] pce, pcplus4e,
    output [4:0] rd_e, rs1_e, rs2_e
);

    // Intermediate signals
    wire regwrited, alusrcd, memwrited, branchd;
    wire [1:0] resultsrcd, immsrcd;
    wire [2:0] alucontrold;
    wire [31:0] rd1_d, rd2_d, imm_extd;

    // Output registers
    reg regwrited_r, alusrcd_r, memwrited_r, branchd_r;
    reg [1:0] resultsrcd_r;
    reg [2:0] alucontrold_r;
    reg [31:0] rd1_d_r, rd2_d_r, imm_extd_r;
    reg [4:0] rd_d_r, rs1_d_r, rs2_d_r;
    reg [31:0] pcd_r, pcplus4d_r;

    control_unit_top control_unit (
        .op(instrd[6:0]),
        .funct3(instrd[14:12]),
        .funct7(instrd[31:25]),
        .regwrite(regwrited),
        .alusrc(alusrcd),
        .memwrite(memwrited),
        .resultsrce(resultsrcd), // ✅ fixed
        .branch(branchd),
        .immsrc(immsrcd),
        .alucontrol(alucontrold)
    );

    register_file regfile (
        .clk(clk),
        .rst(rst),
        .we3(regwritew),
        .wd3(resultw),
        .a1(instrd[19:15]),
        .a2(instrd[24:20]),
        .a3(rd_w),
        .rd1(rd1_d),
        .rd2(rd2_d)
    );

    sign_extend signext (
        .in(instrd),
        .immsrc(immsrcd),
        .imm_ext(imm_extd)
    );

    // Clocked stage registers
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            regwrited_r <= 0; alusrcd_r <= 0; memwrited_r <= 0; resultsrcd_r <= 0;
            branchd_r <= 0; alucontrold_r <= 0;
            rd1_d_r <= 0; rd2_d_r <= 0; imm_extd_r <= 0; rd_d_r <= 0;
            rs1_d_r <= 0; rs2_d_r <= 0;
            pcd_r <= 0; pcplus4d_r <= 0;
        end else begin
            regwrited_r <= regwrited; alusrcd_r <= alusrcd; memwrited_r <= memwrited;
            resultsrcd_r <= resultsrcd; branchd_r <= branchd;
            alucontrold_r <= alucontrold;
            rd1_d_r <= rd1_d; rd2_d_r <= rd2_d; imm_extd_r <= imm_extd;
            rd_d_r <= instrd[11:7];
            rs1_d_r <= instrd[19:15];
            rs2_d_r <= instrd[24:20];
            pcd_r <= pcd; pcplus4d_r <= pcplus4d;
        end
    end

    // Output assignments
    assign regwritee = regwrited_r;
    assign alusrce = alusrcd_r;
    assign memwritee = memwrited_r;
    assign resultsrce = resultsrcd_r;
    assign branche = branchd_r;
    assign alucontrole = alucontrold_r;
    assign rd1_e = rd1_d_r;
    assign rd2_e = rd2_d_r;
    assign imm_exte = imm_extd_r;
    assign rd_e = rd_d_r;
    assign rs1_e = rs1_d_r;
    assign rs2_e = rs2_d_r;
    assign pce = pcd_r;
    assign pcplus4e = pcplus4d_r;
    assign immsrce = immsrcd;

endmodule


//declare test bench code for simulation
//------------------- TESTBENCH FOR DECODE CYCLE -------------------
`timescale 1ns/1ps
module decode_cycle_tb;

  reg clk, rst;
  reg regwritew;
  reg [4:0] rd_w;
  reg [31:0] instrd, pcd, pcplus4d, resultw;
  wire [4:0] rd_e;
  wire regwritee, alusrce, memwritee, branche;
  wire [1:0] resultsrce;
  wire [2:0] alucontrole;
  wire [31:0] rd1_e, rd2_e, imm_exte, pce, pcplus4e;
  wire [4:0]  rs1_e,rs2_e;

  // Instantiate the unit under test (UUT)
  decode_cycle uut (
    .clk(clk),
    .rst(rst),
    .instrd(instrd),
    .pcd(pcd),
    .pcplus4d(pcplus4d),
    .regwritew(regwritew),
    .rd_w(rd_w),
    .resultw(resultw),
    .regwritee(regwritee),
    .alusrce(alusrce),
    .memwritee(memwritee),
    .resultsrce(resultsrce),
    .branche(branche),
    .alucontrole(alucontrole),
    .rd1_e(rd1_e),
    .rd2_e(rd2_e),
    .imm_exte(imm_exte),
    .rd_e(rd_e),
    .rs1_e(rs1_e),.rs2_e(rs2_e),
    .pce(pce),
    .pcplus4e(pcplus4e)
  );

  // Clock generation
  always #5 clk = ~clk;

  // Monitor key signals
  initial begin
    $dumpfile("decode_cycle.vcd");
    $dumpvars(0, decode_cycle_tb);

    $monitor("T=%0t | instrd=%h | rd1_e=%h | rd2_e=%h | alucontrole=%b | regwritee=%b", 
              $time, instrd, rd1_e, rd2_e, alucontrole, regwritee);
  end

  // Stimulus
  initial begin
    clk = 0;
    rst = 0;
    regwritew = 0;
    rd_w = 5'd0;
    resultw = 32'h0;
    instrd = 32'h00000013; // ADDI x0, x0, 0
    pcd = 32'h00000000;
    pcplus4d = 32'h00000004;

    rst = 0;
    #5;
    rst=1;
/*
    // Test case 1: ADD x1, x2, x3 (R-type)
    #20 instrd = 32'b0000000_00011_00010_000_00001_0110011; // add x1, x2, x3

    // Test case 2: AND x4, x5, x6
    #20 instrd = 32'b0000000_00110_00101_111_00100_0110011; // and x4, x5, x6

    // Test case 3: OR x7, x8, x9
    #20 instrd = 32'b0000000_01001_01000_110_00111_0110011; // or x7, x8, x9

    // Test case 4: SUB x10, x11, x12
    #20 instrd = 32'b0100000_01100_01011_000_01010_0110011; // sub x10, x11, x12

    // Extra NOP to latch last values
    #20 instrd = 32'h00000013; // ADDI x0, x0, 0 (NOP)

    #20 $finish;
    */
  end

endmodule

