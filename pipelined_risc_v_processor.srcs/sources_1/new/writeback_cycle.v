`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/25/2025 01:30:13 PM
// Design Name: 
// Module Name: writeback_cycle
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
//declaring a mux which is required for write back cycle
// 3-to-1 MUX for writeback stage
module mux_3_by_1(a, b, c, s, d);
    input [31:0] a, b, c;
    input [1:0] s;
    output [31:0] d;

    assign d = (s == 2'b00) ? a :
               (s == 2'b01) ? b :
               (s == 2'b10) ? c :
               32'h00000000; // default
endmodule

module writeback_cycle(
    input clk, rst,
    input [31:0] aluresultw,
    input [31:0] pcplus4w,
    input [31:0] readdataw,
    input  [1:0]resultsrcw,
    output  [31:0] resultw
);

mux_3_by_1 result_mux (    
    .a(aluresultw),
    .b(readdataw),
    .c(pcplus4w),
    .s(resultsrcw),
    .d(resultw)
);
endmodule

//declaration of test bench for simulation
`timescale 1ns / 1ps

module writeback_cycle_tb;
  // Inputs
  reg clk;
  reg rst;
  reg [31:0] aluresultw;
  reg [31:0] pcplus4w;
  reg [31:0] readdataw;
  reg [1:0] resultsrcw;   // Must be 2 bits for mux_3_by_1

  // Output
  wire [31:0] resultw;

  // Instantiate the Unit Under Test (UUT)
  writeback_cycle uut (
    .clk(clk),
    .rst(rst),
    .aluresultw(aluresultw),
    .pcplus4w(pcplus4w),
    .readdataw(readdataw),
    .resultsrcw(resultsrcw),
    .resultw(resultw)
  );

  initial begin
    // Open VCD for GTKWave
    $dumpfile("writeback_cycle.vcd");
    $dumpvars(0, writeback_cycle_tb);

    // Initialize inputs
    clk = 0;
    rst = 0;
    aluresultw = 32'hAAAA_AAAA;
    readdataw = 32'hBBBB_BBBB;
    pcplus4w = 32'hCCCC_CCCC;
    resultsrcw = 2'b00;

    #10 rst = 1;

    // Test ALU result select
    #10 resultsrcw = 2'b00; // Expect resultw = aluresultw
    #10 resultsrcw = 2'b01; // Expect resultw = readdataw
    #10 resultsrcw = 2'b10; // Expect resultw = pcplus4w
    #10 resultsrcw = 2'b11; // Undefined, should be 0 per mux default
    
    #10;
@(posedge clk);  // wait one extra clock
$display("Resultw (after 1 cycle) = %h", resultw);


    #20 $finish;
  end

  always #5 clk = ~clk; // Clock toggling every 5ns

endmodule
