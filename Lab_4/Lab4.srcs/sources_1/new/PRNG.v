`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// UART Transmitter Module (uart_tx)
// Modified to match board constraints:
//   • Clock input renamed to sysclk
//   • UART TX output renamed to uart_txd_in
//////////////////////////////////////////////////////////////////////////////////
module seed_provider (
    output [31:0] C1_seed,
    output [31:0] C2_seed,
    output [31:0] L_seed
);
    // Example static assignment or logic to generate seeds
    assign C1_seed = 32'hA5A5A5A5;
    assign C2_seed = 32'h5A5A5A5A;
    assign L_seed  = 32'hDEADBEEF;
endmodule

module debouncer (
    input  clk,
    input  noisy,
    output reg clean
);
    parameter WIDTH = 20; // debounce period bits

    reg [WIDTH-1:0] counter = 0;
    reg             sync_0 = 0, sync_1 = 0;

    always @(posedge clk) begin
        // Double flip-flop synchronization
        sync_0 <= noisy;
        sync_1 <= sync_0;

        // Debounce logic
        if (sync_1 == clean) begin
            counter <= 0;
        end else begin
            counter <= counter + 1;
            if (counter == {WIDTH{1'b1}})
                clean <= sync_1;
        end
    end
endmodule


module uart_tx (
    input        clk,
    input        rst,
    input        send,
    input  [7:0] data_in,
    output reg   tx,
    output reg   busy
);
    parameter CLOCK_FREQ = 50000000;
    parameter BAUD_RATE  = 115200;
    localparam CLKS_PER_BIT = CLOCK_FREQ / BAUD_RATE;

    reg [15:0] clk_count = 0;
    reg [3:0]  bit_index = 0;
    reg [9:0]  shift_reg = 10'b1111111111;
    reg        sending = 0;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            clk_count  <= 0;
            bit_index  <= 0;
            shift_reg  <= 10'b1111111111;
            tx         <= 1;
            busy       <= 0;
            sending    <= 0;
        end else begin
            if (send && !sending) begin
                shift_reg  <= {1'b1, data_in, 1'b0};
                sending    <= 1;
                busy       <= 1;
                clk_count  <= 0;
                bit_index  <= 0;
            end else if (sending) begin
                if (clk_count < CLKS_PER_BIT - 1) begin
                    clk_count <= clk_count + 1;
                end else begin
                    clk_count <= 0;
                    tx <= shift_reg[bit_index];
                    if (bit_index < 9) begin
                        bit_index <= bit_index + 1;
                    end else begin
                        sending <= 0;
                        busy <= 0;
                        tx <= 1;
                    end
                end
            end
        end
    end
endmodule

//////////////////////////////////////////////////////////////////////////////////
// Chua RNG Module (chua_rng)
// Digital simulation of Chua's circuit using Euler integration.
// Generates a 16-bit random number by extracting the lower 16 bits of v1.
// Fixed-point format: Q16 (32-bit signed; 16 integer and 16 fractional bits)
//////////////////////////////////////////////////////////////////////////////////
module chua_rng (
    input              sysclk,
    input              rst,
    output reg [15:0]  rnd
);
    reg signed [31:0] v1;
    reg signed [31:0] v2;
    reg signed [31:0] iL;

    wire [31:0] C1_seed;
    wire [31:0] C2_seed;
    wire [31:0] L_seed;

    seed_provider seeds (
        .C1_seed(C1_seed),
        .C2_seed(C2_seed),
        .L_seed(L_seed)
    );

    parameter signed [31:0] inv_C1 = 32'd6554;
    parameter signed [31:0] inv_C2 = 32'd655;
    parameter signed [31:0] inv_L  = 32'd3641;
    parameter signed [31:0] R  = 32'd10000;
    parameter signed [31:0] dt = 32'd66;
    parameter signed [31:0] Ga = -32'd49152;
    parameter signed [31:0] Gb =  32'd32768;
    parameter signed [31:0] B  =  32'd8192;

    function signed [31:0] nonlinear;
        input signed [31:0] x;
        begin
            if (x > B)
                nonlinear = (Gb * x) >>> 16;
            else if (x < -B)
                nonlinear = (Gb * x) >>> 16;
            else
                nonlinear = (Ga * x) >>> 16;
        end
    endfunction

    reg signed [31:0] dv1, dv2, diL;
    reg signed [31:0] diff, term;
    reg               initialized = 0;

    always @(posedge sysclk) begin
        if (rst || !initialized) begin
            v1  <= C1_seed << 16;
            v2  <= C2_seed << 16;
            iL  <= L_seed  << 16;
            rnd <= 16'h0000;
            initialized <= 1;
        end else begin
            diff = v1 - v2;
            term = diff / R;
            dv1 = (inv_C1 * (term - nonlinear(v1))) >> 16;
            dv2 = (inv_C2 * ((-diff) / R + iL)) >> 16;
            diL = (inv_L * (-v2)) >> 16;

            v1  <= v1  + ((dt * dv1) >> 16);
            v2  <= v2  + ((dt * dv2) >> 16);
            iL  <= iL  + ((dt * diL) >> 16);

            rnd <= v1[15:0];
        end
    end
endmodule
//////////////////////////////////////////////////////////////////////////////////
// Top-Level Module (chua_uart_rng)
// Instantiates the Chua RNG and a UART transmitter.
// It latches a new 16-bit random number from the Chua simulation and sends it out via UART
// as two sequential 8-bit bytes (high byte first, then low byte).
// Ports are renamed to match the XDC constraint file:
//  - Clock input is now sysclk.
//  - UART TX output is now uart_txd_in.
//  - A dummy UART RX input, uart_rxd_out, is added.
//////////////////////////////////////////////////////////////////////////////////
module chua_uart_rng (
    input       clk,           // 50 MHz clock
    input       rst,           // button trigger
    output      uart_rxd_out,  // renamed TX
    input       dummy_uart_rx  // unused
);
    wire [15:0] rnd;
    reg  [15:0] random_value;

    reg        send;
    reg [7:0]  data_in;
    wire       busy;

    reg rst_prev = 0;
    wire rst_pulse = rst & ~rst_prev;

    always @(posedge clk) rst_prev <= rst;

    chua_rng rng_inst (
        .sysclk(clk),
        .rst(rst_pulse), // only on rising edge
        .rnd(rnd)
    );

    uart_tx uart_tx_inst (
        .clk(clk),
        .rst(1'b0),
        .send(send),
        .data_in(data_in),
        .tx(uart_rxd_out),
        .busy(busy)
    );

    localparam STATE_IDLE       = 0;
    localparam STATE_SEND_HIGH  = 1;
    localparam STATE_WAIT_HIGH  = 2;
    localparam STATE_SEND_LOW   = 3;
    localparam STATE_WAIT_LOW   = 4;

    reg [2:0] state = STATE_IDLE;

    always @(posedge clk) begin
        case (state)
            STATE_IDLE: begin
                send <= 0;
                if (rst_pulse) begin
                    random_value <= rnd;
                    data_in <= rnd[15:8];
                    send <= 1;
                    state <= STATE_SEND_HIGH;
                end
            end
            STATE_SEND_HIGH: begin
                send <= 0;
                state <= STATE_WAIT_HIGH;
            end
            STATE_WAIT_HIGH: begin
                if (!busy) begin
                    data_in <= random_value[7:0];
                    send <= 1;
                    state <= STATE_SEND_LOW;
                end
            end
            STATE_SEND_LOW: begin
                send <= 0;
                state <= STATE_WAIT_LOW;
            end
            STATE_WAIT_LOW: begin
                if (!busy)
                    state <= STATE_IDLE;
            end
            default: state <= STATE_IDLE;
        endcase
    end
endmodule