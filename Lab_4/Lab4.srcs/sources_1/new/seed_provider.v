`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2025 16:27:45
// Design Name: 
// Module Name: seed_provider
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


module seed_provider (
    output [31:0] C1_seed,
    output [31:0] C2_seed,
    output [31:0] L_seed
);
    assign C1_seed = 32'hA5A5A5A5;
    assign C2_seed = 32'h5A5A5A5A;
    assign L_seed  = 32'hDEADBEEF;
endmodule

