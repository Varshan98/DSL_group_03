// Testbench for Chua's circuit random number generator
module chua_rng_tb;

    // Parameters
    localparam CLOCK_PERIOD = 10;    // Clock period in time units
    localparam TOTAL_STEPS = 200000; // Total number of simulation steps
    localparam SKIP_STEPS = 30000;   // Number of initial steps to skip

    // Signals
    reg clk = 0;
    reg reset = 0;
    wire [15:0] random_bits;
    integer file_handle;
    reg [31:0] step_counter = 0;

    // Clock generation
    always #(CLOCK_PERIOD / 2) clk = ~clk;

    // Instantiate the Device Under Test (DUT)
    chua_rng dut (
        .clk(clk),
        .reset(reset),
        .random_bits(random_bits)
    );

    // Simulation control
    initial begin
        // Open file for writing binary data
        file_handle = $fopen("random_output.bin", "wb");
        if (file_handle == 0) begin
            $display("Error: Unable to open file 'random_output.bin' for writing.");
            $finish;
        end

        // Apply reset
        reset = 1;
        # (CLOCK_PERIOD * 2);
        reset = 0;

        // Run simulation for TOTAL_STEPS clock cycles
        # (CLOCK_PERIOD * TOTAL_STEPS);

        // Close the file
        $fclose(file_handle);
        $display("Simulation completed. Random bits saved to 'random_output.bin'");
        $finish;
    end

    // Step counter and file writing
    always @(posedge clk) begin
        if (reset) begin
            step_counter <= 0;
        end else begin
            step_counter <= step_counter + 1;
            if (step_counter >= SKIP_STEPS) begin
                // Write 16-bit random_bits as two bytes (big-endian)
                $fwrite(file_handle, "%c%c", random_bits[15:8], random_bits[7:0]);
            end
        end
    end

endmodule
