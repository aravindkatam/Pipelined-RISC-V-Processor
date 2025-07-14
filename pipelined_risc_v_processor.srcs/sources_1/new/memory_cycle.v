`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/25/2025 10:50:05 AM
// Design Name: 
// Module Name: memory_cycle
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
//declaration of data_memory module which is used for memory_cycle module 
module data_memory(clk, rst, we, wd, a, rd);
    input clk, rst, we;
    input [31:0] a, wd;
    output [31:0] rd;

    reg [31:0] mem [1023:0];

    always @(posedge clk) begin
        if (we)
            mem[a[31:2]] <= wd;
    end

    assign rd = (~rst) ? 32'd0 : mem[a[31:2]];

    initial begin
        mem[0] = 32'h00000000;
    end
    initial begin
    mem[0] = 32'h00000000;
    mem[1] = 32'h00000001;
    mem[2] = 32'h00000002;
    mem[3] = 32'h00000003;
    mem[4] = 32'h00000004;
    mem[5] = 32'h00000005;
    mem[6] = 32'h00000006;
    mem[7] = 32'h00000007;
end

endmodule


// ===================== memory_cycle.v =====================
module memory_cycle(clk,rst,
regwritem,memwritem,resultsrcm,rd_m,pcplus4m,writedatam,aluresultm,
regwritew,resultsrcw,rd_w,pcplus4w,readdataw,aluresultw);

input clk,rst,regwritem,memwritem;
input [1:0]resultsrcm;
input [4:0]rd_m;
input [31:0]pcplus4m,writedatam,aluresultm;

output regwritew;
output [1:0]resultsrcw;
output [4:0] rd_w;
output [31:0] pcplus4w, readdataw, aluresultw;

wire [31:0]readdatam;

reg regwritem_r;
reg [1:0]resultsrcm_r;
reg [4:0] rd_m_r;
reg [31:0]pcplus4m_r,readdatam_r,aluresultm_r;

data_memory dmem(.clk(clk),.rst(rst),.we(memwritem),.wd(writedatam),.a(aluresultm),.rd(readdatam));

always@(posedge clk or negedge rst) begin
    if(!rst) begin
        regwritem_r    <= 1'b0;
        resultsrcm_r   <= 2'b00;
        rd_m_r         <= 5'd0;
        pcplus4m_r     <= 32'd0;
        readdatam_r    <= 32'd0;
        aluresultm_r   <= 32'd0;
    end else begin
        regwritem_r    <= regwritem;
        resultsrcm_r   <= resultsrcm;
        rd_m_r         <= rd_m;
        pcplus4m_r     <= pcplus4m;
        readdatam_r    <= readdatam;
        aluresultm_r   <= aluresultm;
    end
end

assign regwritew   = regwritem_r;
assign resultsrcw  = resultsrcm_r;
assign rd_w        = rd_m_r;
assign pcplus4w    = pcplus4m_r;
assign readdataw   = readdatam_r;
assign aluresultw  = aluresultm_r;
endmodule



//generating test bench for simulation
`timescale 1ns / 1ps

module memory_cycle_tb;

  reg clk, rst;
  reg regwritem, memwritem;
  reg [1:0] resultsrcm;
  reg [4:0] rd_m;
  reg [31:0] pcplus4m, writedatam, aluresultm;

  wire regwritew;
  wire [1:0]resultsrcw;
  wire [4:0] rd_w;
  wire [31:0] pcplus4w, readdataw, aluresultw;

  // Instantiate the memory_cycle module
  memory_cycle uut (
    .clk(clk),
    .rst(rst),
    .regwritem(regwritem),
    .memwritem(memwritem),
    .resultsrcm(resultsrcm),
    .rd_m(rd_m),
    .pcplus4m(pcplus4m),
    .writedatam(writedatam),
    .aluresultm(aluresultm),
    .regwritew(regwritew),
    .resultsrcw(resultsrcw),
    .rd_w(rd_w),
    .pcplus4w(pcplus4w),
    .readdataw(readdataw),
    .aluresultw(aluresultw)
  );

  // Clock generation
  always #5 clk = ~clk;

  initial begin
    $dumpfile("memory_cycle.vcd");
    $dumpvars(0, memory_cycle_tb);

    // Initialize inputs
    clk = 0;
    rst = 0;
    regwritem = 0;
    memwritem = 0;
    resultsrcm = 0;
    rd_m = 5'd3;
    pcplus4m = 32'h00000004;
    writedatam = 32'hDEADBEEF;
    aluresultm = 32'd5; // Write into memory[5]

    // Reset active
    #10 rst = 1;

    // Write to memory[5]
    #10 memwritem = 1;
        writedatam = 32'h12345678;
        aluresultm = 32'd5;
        regwritem = 1;
        resultsrcm = 1;

    // Disable write, check if value is latched
    #10 memwritem = 0;

    // Wait one more clock to confirm outputs
    #10;

    // Print results
    $display("readdataw    = %h", readdataw);
    $display("aluresultw   = %h", aluresultw);
    $display("pcplus4w     = %h", pcplus4w);
    $display("rd_w         = %d", rd_w);
    $display("regwritew    = %b", regwritew);
    $display("resultsrcw   = %b", resultsrcw);

    #10 $finish;
  end

endmodule

