`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2025 16:22:33
// Design Name: 
// Module Name: debouncer
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


module debouncer (
    input  clk,
    input  noisy,
    output reg clean
);
    parameter WIDTH = 20; // Number of bits for debounce time (adjust as needed)

    reg [WIDTH-1:0] counter = 0;
    reg             sync_0 = 0, sync_1 = 0;

    always @(posedge clk) begin
        // Double flip-flop synchronizer
        sync_0 <= noisy;
        sync_1 <= sync_0;

        // Counter for debounce logic
        if (sync_1 == clean) begin
            counter <= 0;
        end else begin
            counter <= counter + 1;
            if (counter == {WIDTH{1'b1}})
                clean <= sync_1;
        end
    end
endmodule

