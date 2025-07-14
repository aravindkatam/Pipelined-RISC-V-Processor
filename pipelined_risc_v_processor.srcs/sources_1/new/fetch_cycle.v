`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/22/2025 09:54:17 AM
// Design Name: 
// Module Name: fetch_cycle
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
//mux module
module mux (a,b,s,c);

    input [31:0]a,b;
    input s;
    output [31:0]c;

    assign c = (~s) ? a : b ;
    
endmodule



//program_couter module
module pc_module(clk,rst,pc,pc_next);
input clk,rst;
input [31:0]pc_next;
output reg [31:0]pc;

always@(posedge clk) begin
if(~rst)
    pc<=32'h0;
else 
    pc<=pc_next;
end
endmodule


//instruction memory module
module instruction_memory(rst,a,rd);

  input rst;
  input [31:0]a;
  output [31:0]rd;

  reg [31:0] mem [1023:0];
  
  assign rd = (rst == 1'b0) ? {32{1'b0}} : mem[a[31:2]];
  initial begin
    $readmemh("memfile.hex",mem);
end
/*
  initial begin
    //mem[0] = 32'hFFC4A303;
    //mem[1] = 32'h00832383;
    // mem[0] = 32'h0064A423;
    // mem[1] = 32'h00B62423;
    mem[0] = 32'h0062E233;
    // mem[1] = 32'h00B62423;

  end
*/
endmodule


//pc_adder module
module pc_adder (a,b,c);
input [31:0]a,b;
output [31:0]c;
assign c = a + b;
    
endmodule





module fetch_cycle( clk,rst,pcsrce,pctargete,instrd,pcd,pcplus4d);
//declare inputs and outputs in fetch stage
input clk,rst;
input pcsrce;
input[31:0]pctargete;

output [31:0]instrd;
output [31:0]pcd,pcplus4d;

//declare intermidiate outputs
wire [31:0]pcf_bar, pcf,pcplus4f;
wire [31:0]instrf;

//declartion of registers
reg [31:0]instrf_reg;
reg [31:0]pcf_reg,pcplus4f_reg;

//intiation of modules
//declare pc mux 
mux pc_mux(.a(pcplus4f),.b(pctargete),.s(pcsrce),.c(pcf_bar));

//declare pc_counter
 pc_module program_counter(.clk(clk),.rst(rst),.pc(pcf),.pc_next(pcf_bar));
 
 //declare instruction memory
 instruction_memory imem(.rst(rst),.a(pcf),.rd(instrf));


//declare pc_adder
pc_adder add4(.a(pcf),.b(32'h00000004),.c(pcplus4f));


//fetch cycle reginster logic
always@(posedge clk or negedge rst) begin
if(rst == 1'b0) begin
instrf_reg <= 32'h00000000;
pcf_reg <= 32'h00000000;
pcplus4f_reg <= 32'h00000000;
end
else begin
instrf_reg <= instrf;
pcf_reg <= pcf;
pcplus4f_reg <= pcplus4f;
end
end 

//assigning registers value to the output ports
assign instrd = (rst == 1'b0)? 32'h00000000 : instrf_reg;
assign pcd = (rst == 1'b0)? 32'h00000000 : pcf_reg;
assign  pcplus4d = (rst == 1'b0)? 32'h00000000 : pcplus4f_reg;

endmodule



//generating testbench for endmodule
`timescale 1ns / 1ps

module fetch_cycle_tb;

    reg clk = 0;
    reg rst;
    reg pcsrce;
    reg [31:0] pctargete;
    wire [31:0] instrd, pcd, pcplus4d;

    // Instantiate your fetch_cycle module
    fetch_cycle uut (
        .clk(clk),
        .rst(rst),
        .pcsrce(pcsrce),
        .pctargete(pctargete),
        .instrd(instrd),
        .pcd(pcd),
        .pcplus4d(pcplus4d)
    );

    // Clock generation - 10ns period
    always #5 clk = ~clk;

    initial begin
        // Initialize signals
        rst = 1;
        pcsrce = 0;
        pctargete = 32'd0;

        // Step 1: Apply reset (active low)
        #5; rst = 0;
        #10; rst = 1;

        // Step 2: Wait a few cycles and observe PC, instruction
        #50;

        // Step 3: Apply branch (simulate branch instruction taken)
        pcsrce = 1;
        pctargete = 32'h00000010;  // Jump to instruction at address 16
        #10; pcsrce = 0;

        // Wait to observe new fetch
        #50;

        $display("\n========== FETCH CYCLE OUTPUT ==========");
        $display("Time        = %0t", $time);
        $display("PC          = %h", pcd);
        $display("Instruction = %h", instrd);
        $display("PC+4        = %h", pcplus4d);
        $display("========================================\n");

        $finish;
    end
endmodule
