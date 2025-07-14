`timescale 1ns / 1ps  
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
//  
// Create Date: 06/25/2025 07:12:06 PM
// Design Name: 
// Module Name: pipeline_top
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



module pipeline_top(clk,rst);

//declaring inputs and outputs
input clk,rst;


//declaration of intermediate outputs
// Declaration of Interim Wires
    wire pcsrce,regwritew,regwritee,alusrce, memwritee, 
    branche,regwritem, memwritem;
    wire [1:0] resultsrce,resultsrcm, resultsrcw;
    wire [2:0] alucontrole;
    wire [4:0] rd_e, rd_m, rd_w;
    wire [31:0] pctargete, instrd, pcd,pcplus4d, resultw,rd1_e,rd2_e,imm_exte,
    pce, pcplus4e,pcplus4m,writedatam, aluresultm;
    wire [31:0] pcplus4w, aluresultw, readdataw;
    wire [4:0] rs1_e, rs2_e;
    wire [1:0] forwardae, forwardbe;
    


//instatiation of modules
//fetch stage
fetch_cycle  fetch( .clk(clk),.rst(rst),.pcsrce(pcsrce),.pctargete(pctargete),
.instrd(instrd),.pcd(pcd),.pcplus4d(pcplus4d));


//decode stage
decode_cycle  decode(.clk(clk),.rst(rst),.instrd(instrd),
.pcd(pcd),.pcplus4d(pcplus4d),.regwritew(regwritew),.rd_w(rd_w),
.resultw(resultw),.regwritee(regwritee),.alusrce(alusrce),.memwritee(memwritee),
.resultsrce(resultsrce),.branche(branche),.alucontrole(alucontrole),
.rd1_e(rd1_e),.rd2_e(rd2_e),.imm_exte(imm_exte),.rd_e(rd_e),
.pce(pce),.pcplus4e(pcplus4e),.rs1_e(rs1_e),.rs2_e(rs2_e));


//execute stage
execute_cycle  execute(.clk(clk),.rst(rst),.regwritee(regwritee),
.alusrce(alusrce),.memwritee(memwritee),.resultsrce(resultsrce),.branche(branche),
.alucontrole(alucontrole),.rd1_e(rd1_e),.rd2_e(rd2_e),.imm_exte(imm_exte),.rd_e(rd_e),
.pce(pce),.pcplus4e(pcplus4e),.pcsrce(pcsrce),.pctargete(pctargete),
.regwritem(regwritem),.memwritem(memwritem),.resultsrcm(resultsrcm),.rd_m(rd_m),
.pcplus4m(pcplus4m),.writedatam(writedatam),.aluresultm(aluresultm),.resultw(resultw),
.forwarda_e(forwardae),.forwardb_e(forwardbe));


//memory stage
 memory_cycle  memory(.clk(clk),.rst(rst),
.regwritem(regwritem),.memwritem(memwritem),.resultsrcm(resultsrcm),.rd_m(rd_m),.
pcplus4m(pcplus4m),.writedatam(writedatam),.aluresultm(aluresultm),
.regwritew(regwritew),.resultsrcw(resultsrcw),.rd_w(rd_w),
.pcplus4w(pcplus4w),.readdataw(readdataw),.aluresultw(aluresultw));


//writeback stage
writeback_cycle   writeback (.clk(clk),.rst(rst),
.aluresultw(aluresultw),.pcplus4w(pcplus4w),
.readdataw(readdataw),.resultsrcw(resultsrcw),.resultw(resultw));

// hazard_unit stage
hazard_unit forwarding_block (.rst(rst),.regwritem(regwritem),.regwritew(regwritew),
.rd_m(rd_m),.rd_w(rd_w),.rs1_e(rs1_e),.rs2_e(rs2_e),.forwardae(forwardae),.forwardbe(forwardbe));

endmodule


//declaration of test bench for simulation
`timescale 1ns / 1ps

module pipeline_top_tb;

  reg clk = 0;
  reg rst;

  // Instantiate pipeline_top
  pipeline_top dut (
    .clk(clk),
    .rst(rst)
  );

  // Clock generation
  always #5 clk = ~clk;

  initial begin
    $display("==== PIPELINE TOP TEST START ====");

    // Reset pulse
    rst = 1; #10;
    rst = 0; #10;
    rst = 1;

    // Wait and observe waveforms
    #100;

    $display("==== PIPELINE TOP TEST FINISH ====");
    $finish;
  end

endmodule
