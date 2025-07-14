`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/28/2025 08:11:04 PM
// Design Name: 
// Module Name: hazard_unit
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
///////////////////////////////////////////////////////////////////////////////// 

module hazard_unit(
    input rst,
    input regwritem, regwritew,
    input [4:0] rd_m, rd_w, rs1_e, rs2_e,
    output reg [1:0] forwardae, forwardbe
);

    always @(*) begin
        if (!rst) begin
            forwardae = 2'b00;
            forwardbe = 2'b00;
        end else begin
            // Forwarding logic for A
            if ((regwritem == 1'b1) && (rd_m != 5'd0) && (rd_m == rs1_e))
                forwardae = 2'b10;
            else if ((regwritew == 1'b1) && (rd_w != 5'd0) && (rd_w == rs1_e))
                forwardae = 2'b01;
            else
                forwardae = 2'b00;

            // Forwarding logic for B
            if ((regwritem == 1'b1) && (rd_m != 5'd0) && (rd_m == rs2_e))
                forwardbe = 2'b10;
            else if ((regwritew == 1'b1) && (rd_w != 5'd0) && (rd_w == rs2_e))
                forwardbe = 2'b01;
            else
                forwardbe = 2'b00;
        end
    end

endmodule


//testbench for simulation 
`timescale 1ns / 1ps

module hazard_unit_tb;

  reg rst;
  reg regwritem, regwritew;
  reg [4:0] rd_m, rd_w, rs1_e, rs2_e;
  wire [1:0] forwardae, forwardbe;

  hazard_unit uut (
    .rst(rst), .regwritem(regwritem), .regwritew(regwritew),
    .rd_m(rd_m), .rd_w(rd_w), .rs1_e(rs1_e), .rs2_e(rs2_e),
    .forwardae(forwardae), .forwardbe(forwardbe)
  );

  initial begin
    $display("=== HAZARD UNIT TEST ===");

    // Reset active
    rst = 0; regwritem = 0; regwritew = 0;
    rd_m = 5'd0; rd_w = 5'd0; rs1_e = 5'd0; rs2_e = 5'd0;
    #10 rst = 1;

    // Case 1: Forward from MEM to EX stage
    regwritem = 1; rd_m = 5'd3; rs1_e = 5'd3; rs2_e = 5'd1;
    regwritew = 0; rd_w = 5'd0;
    #10;
    $display("MEM->EX forwardAE=%b (expect 10), forwardBE=%b (expect 00)", forwardae, forwardbe);

    // Case 2: Forward from WB to EX stage
    regwritem = 0; rd_m = 5'd0; 
    regwritew = 1; rd_w = 5'd2; rs1_e = 5'd2; rs2_e = 5'd2;
    #10;
    $display("WB->EX forwardAE=%b (expect 01), forwardBE=%b (expect 01)", forwardae, forwardbe);

    // Case 3: No forwarding
    regwritew = 1; rd_w = 5'd4; rs1_e = 5'd1; rs2_e = 5'd1;
    #10;
    $display("NO forwardAE=%b (expect 00), forwardBE=%b (expect 00)", forwardae, forwardbe);

    $display("=== HAZARD UNIT TEST COMPLETE ===");
    $finish;
  end

endmodule
