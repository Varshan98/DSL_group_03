`timescale 1ns / 1ps
module chua_rng_tb;

    // Inputs
    reg clk;
    reg reset;
    reg [11:0] seed_C1, seed_C2, seed_L;

    // Output
    wire [15:0] random_num;

    // Instantiate the Unit Under Test (UUT)
    chua_rng uut (
        .clk(clk),
        .reset(reset),
        .seed_C1(seed_C1),
        .seed_C2(seed_C2),
        .seed_L(seed_L),
        .random_num(random_num)
    );

    // Clock generation: 100MHz
    initial clk = 0;
    always #5 clk = ~clk;  // Toggle every 5ns

    integer i;

    initial begin
        // Correct assignment style
        reset = 1;
        seed_C1 = 12'd156;
        seed_C2 = 12'd432;
        seed_L  = 12'd117;

        #30;  // Keep reset high for 30ns (3 clock cycles)
        reset = 0;

        $display("Time (ns)\tRandom Output");
        $display("-----------------------------");

        // Run simulation for 500 cycles
        for (i = 0; i < 500; i = i + 1) begin
            #10; // wait one clock cycle
            if (i > 10)
                $display("%8dns\t%h", $time, random_num);
        end

        $display("-----------------------------");
        $finish;
    end

endmodule
