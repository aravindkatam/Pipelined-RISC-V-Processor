`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/23/2025 12:24:25 PM
// Design Name: 
// Module Name: execute_cycle
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


// ================= ALU Module =================
module alu(a, b, result, alucontrol, overflow, carry, zero, negative);
    input [31:0] a, b;
    input [2:0] alucontrol;
    output [31:0] result;
    output carry, overflow, zero, negative;

    wire [31:0] sum;
    wire cout;

    assign sum = (alucontrol[0] == 1'b0) ? a + b : (a + (~b + 1));
    assign result = (alucontrol == 3'b000) ? sum :
                    (alucontrol == 3'b001) ? sum :
                    (alucontrol == 3'b010) ? a & b :
                    (alucontrol == 3'b011) ? a | b :
                    (alucontrol == 3'b101) ? {31'b0, sum[31]} :
                    32'b0;

    assign overflow = ((sum[31] ^ a[31]) & 
                      (~(alucontrol[0] ^ b[31] ^ a[31])) &
                      (~alucontrol[1]));
    assign carry = 0; // fixed unused cout
    assign zero = (result == 32'b0);
    assign negative = result[31];
endmodule

// ================= 2:1 MUX =================
module mux1(a, b, s, c);
    input [31:0] a, b;
    input s;
    output [31:0] c;
    assign c = (s == 1'b0) ? a : b;
endmodule

// ================= 3:1 MUX =================
module mux_3_by_1_1(a, b, c, s, d);
    input [31:0] a, b, c;
    input [1:0] s;
    output [31:0] d;
    assign d = (s == 2'b00) ? a :
               (s == 2'b01) ? b :
               (s == 2'b10) ? c : 32'h00000000;
endmodule

// ================= PC ADDER =================
module pc_adder1(a, b, c);
    input [31:0] a, b;
    output [31:0] c;
    assign c = a + b;
endmodule

// ===================== execute_cycle.v =====================
// Only updates to ensure proper reset and clean waveform

module execute_cycle(
    input clk, rst,
    input regwritee, alusrce, memwritee, branche,
    input [1:0] resultsrce,
    input [2:0] alucontrole,
    input [31:0] rd1_e, rd2_e, imm_exte,
    input [4:0] rd_e,
    input [31:0] pce, pcplus4e, resultw,
    input [1:0] forwarda_e, forwardb_e,

    output pcsrce,
    output regwritem, memwritem,
    output [1:0] resultsrcm,
    output [4:0] rd_m,
    output [31:0] pcplus4m, writedatam, aluresultm,
    output [31:0] pctargete
);

    wire [31:0] src_a, src_b_intermediate, src_b;
    wire [31:0] resulte;
    wire zeroe;
    
    reg regwritee_r, memwritee_r;
    reg [1:0] resultsrce_r;
    reg [4:0] rd_e_r;
    reg [31:0] pcplus4e_r, rd2_e_r, resulte_r;

    mux_3_by_1_1 srca_mux (.a(rd1_e), .b(resultw), .c(aluresultm), .s(forwarda_e), .d(src_a));
    mux_3_by_1_1 srcb_mux (.a(rd2_e), .b(resultw), .c(aluresultm), .s(forwardb_e), .d(src_b_intermediate));

    mux1 alu_src_mux (.a(src_b_intermediate), .b(imm_exte), .s(alusrce), .c(src_b));

    alu alu_inst(
        .a(src_a), .b(src_b), .result(resulte),
        .alucontrol(alucontrole), .overflow(), .carry(), .zero(zeroe), .negative()
    );

    pc_adder1 branch_adder(.a(pce), .b(imm_exte), .c(pctargete));



    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            regwritee_r  <= 1'b0;
            memwritee_r  <= 1'b0;
            resultsrce_r <= 2'b00;
            rd_e_r       <= 5'd0;
            pcplus4e_r   <= 32'd0;
            rd2_e_r      <= 32'd0;
            resulte_r    <= 32'd0;
        end else begin
            regwritee_r  <= regwritee;
            memwritee_r  <= memwritee;
            resultsrce_r <= resultsrce;
            rd_e_r       <= rd_e;
            pcplus4e_r   <= pcplus4e;
            rd2_e_r      <= src_b_intermediate;
            resulte_r    <= resulte;
        end
    end

    assign pcsrce     = (rst == 1'b0) ? 1'b0 : (zeroe & branche);
    assign regwritem  = regwritee_r;
    assign memwritem  = memwritee_r;
    assign resultsrcm = resultsrce_r;
    assign rd_m       = rd_e_r;
    assign pcplus4m   = pcplus4e_r;
    assign writedatam = rd2_e_r;
    assign aluresultm = resulte_r;
endmodule




//generating testbench for simulation
`timescale 1ns / 1ps

module execute_cycle_tb;

    // Inputs
    reg clk, rst;
    reg regwritee, alusrce, memwritee, branche;
    reg [1:0] resultsrce;
    reg [2:0] alucontrole;
    reg [31:0] rd1_e, rd2_e, imm_exte;
    reg [4:0] rd_e;
    reg [31:0] pce, pcplus4e, resultw;
    reg [1:0] forwarda_e, forwardb_e;

    // Outputs
    wire pcsrce;
    wire regwritem, memwritem;
    wire [1:0] resultsrcm;
    wire [4:0] rd_m;
    wire [31:0] pcplus4m, writedatam, aluresultm, pctargete;

    // Instantiate DUT
    execute_cycle uut (
        .clk(clk), .rst(rst),
        .regwritee(regwritee), .alusrce(alusrce),
        .memwritee(memwritee), .branche(branche),
        .resultsrce(resultsrce),
        .alucontrole(alucontrole),
        .rd1_e(rd1_e), .rd2_e(rd2_e), .imm_exte(imm_exte),
        .rd_e(rd_e),
        .pce(pce), .pcplus4e(pcplus4e), .resultw(resultw),
        .forwarda_e(forwarda_e), .forwardb_e(forwardb_e),
        .pcsrce(pcsrce),
        .regwritem(regwritem), .memwritem(memwritem),
        .resultsrcm(resultsrcm), .rd_m(rd_m),
        .pcplus4m(pcplus4m), .writedatam(writedatam),
        .aluresultm(aluresultm), .pctargete(pctargete)
    );

    // Clock generation
    always #5 clk = ~clk;

    // Initial
    initial begin
        $dumpfile("execute_cycle_tb.vcd");
        $dumpvars(0, execute_cycle_tb);

        clk = 0;
        rst = 0;

        // Apply reset
        #10 rst = 1;

        // After reset, simulation will work automatically from pipeline using memfile.hex

        // Run long enough to observe all instructions from memory
        #200 $finish;
    end

endmodule


