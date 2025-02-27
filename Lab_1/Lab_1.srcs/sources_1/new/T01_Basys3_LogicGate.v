`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.02.2025 13:12:19
// Design Name: 
// Module Name: T01_Basys3_LogicGate
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


module T01_Basys3_LogicGate(
input [1:0]sw,
output [7:0]led
    );
    //Logic Gates
    assign led[0] =~sw[0];
    assign led[1] =~(~sw[1]);
    assign led[2] = sw[0]&sw[1];
    assign led[3] = sw[0]|sw[1];
    assign led[4] =~(sw[0]|sw[1]);
    assign led[5] =~(sw[0]&sw[1]);
    assign led[6] = sw[0]^sw[1];
    assign led[7] = ~(sw[0]^sw[1]);
     
endmodule
