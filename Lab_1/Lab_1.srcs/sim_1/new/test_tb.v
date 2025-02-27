`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.02.2025 13:26:50
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


module test_tb;
reg [1:0]sw;
wire [7:0]led;

T01_Basys3_LogicGate uut(sw,led);
initial begin
 $dumpfile("tb.vcd");
 $dumpvars(0,test_tb); 
 
#0 sw = 2'b00;
#1 sw = 2'b01;
#1 sw = 2'b10;
#1 sw = 2'b11;
#1 $finish;

   $dumpoff;
   end
endmodule
