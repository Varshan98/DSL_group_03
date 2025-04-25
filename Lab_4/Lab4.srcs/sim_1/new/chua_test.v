`timescale 1ns / 1ps

module testb_chua_rng;

    // DUT I/O
    reg clk;
    reg rstn;
    reg start;
    reg [31:0] seed;
    wire done;
    wire [15:0] rand_val;

    // Instantiate the DUT (Device Under Test)
    chua_rng uut (
        .clk(clk),
        .rstn(rstn),
        .start(start),
        .seed(seed),
        .done(done),
        .rand_val(rand_val)
    );

    // Clock generation (100 MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Main test sequence
    initial begin
        $display("Starting Chua RNG test...");

        // Initial state
        rstn = 0;
        start = 0;
        seed = 32'h12345678;  // You can change this to test other seeds

        // Reset pulse
        #20;
        rstn = 1;

        // Wait then send start pulse
        #20;
        start = 1;
        #10;
        start = 0;

        // Wait for done
        wait(done);
        $display("Done signal received!");
        $display("Random Value: %d (0x%04X)", rand_val, rand_val);

        // Finish
        #10;
        $finish;
    end

endmodule
