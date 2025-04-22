`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2025 16:17:08
// Design Name: 
// Module Name: chua_uart_rng
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
