`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.02.2025 12:28:34
// Design Name: 
// Module Name: logic_gate
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
module logic_gate(
input sw, // 2-bit input (sw[1], sw[0]), use 1 bit only
output led // 8-bit output (led[7:0]), use 1 bit only
);
// Logic gate implementations
assign led = ~sw; // NOT A
//Your Code Here;
//Try to use different methods e.g. assign, instance;
endmodule
//