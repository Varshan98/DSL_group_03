`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.02.2025 12:32:15
// Design Name: 
// Module Name: test_tb
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

//
module tb;
reg sw;
wire led;
logic_gate uut (sw, led);
initial begin
$dumpfile("tb.vcd");
$dumpvars(0,tb);
sw = 1'b0;
#1 sw = 1'b1;
#1 $finish;
$dumpoff;
end
endmodule
//
