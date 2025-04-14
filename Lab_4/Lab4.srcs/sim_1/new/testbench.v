//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.04.2025 12:40:17
// Design Name: 
// Module Name: testbench
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


`timescale 1ns/1ps
module tb_chua_uart_rng;
  // Declare testbench signals.
  reg         sysclk;
  reg         rst;
  wire [15:0] rnd_out;

  // Instantiate the Chua RNG module.
  chua_rng rng_uut (
    .sysclk(sysclk),
    .rst(rst),
    .rnd(rnd_out)
  );

  // Combined initial block using one for loop.
  initial begin
    // Initialize values.
    sysclk = 0;

    rst = 0;

    // Dump all variables for waveform inspection.
    $dumpfile("simulation.vcd");
    $dumpvars(0, tb_chua_uart_rng);

    // Run the simulation loop. 
    // For example, if you want to run for 10,000,000 ns using a 5 ns time step:
    // Total iterations = 10,000,000 ns / 5 ns = 2,000,000 iterations.
    for (integer i = 0; i < 2000000; i = i + 1) begin
      #5;                 // Delay of 5 ns per iteration.
      sysclk = ~sysclk;   // Toggle the clock every 5 ns.

      // Update seed_2 every 10 ns.
      // Since our time step is 5 ns, do it on every second iteration.
      if (i % 2 == 0)


      // Toggle reset every 40 ns.
      // (40 ns / 5 ns = 8 iterations; every 8 iterations, toggle rst).
      if (i % 8 == 0)
        rst = ~rst;
    end

    // End simulation
    $stop;
  end

endmodule
