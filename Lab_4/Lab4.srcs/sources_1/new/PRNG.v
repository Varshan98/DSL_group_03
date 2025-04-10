`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// UART Transmitter Module (uart_tx)
// Modified to match board constraints:
//   • Clock input renamed to sysclk
//   • UART TX output renamed to uart_txd_in
//////////////////////////////////////////////////////////////////////////////////
module uart_tx (
    input        sysclk,       // was clk
    input        rst,
    input        tx_start,         // pulse to start transmission
    input  [7:0] tx_data,          // data to transmit
    output reg   uart_txd_in,      // renamed to match XDC constraint
    output reg   tx_done           // asserted for one cycle when transmission finishes
);
  // Parameters: change these as needed.
  parameter CLOCK_FREQ = 50000000;  // 50 MHz clock
  parameter BAUD_RATE  = 115200;    // desired baud rate
  localparam BAUD_TICK = CLOCK_FREQ / BAUD_RATE;

  reg [15:0] baud_counter;
  reg [3:0]  bit_index;
  reg [9:0]  tx_shift;             // [0]: start, [1]-[8]: data, [9]: stop
  reg        sending;

  always @(posedge sysclk or posedge rst) begin
    if (rst) begin
      baud_counter <= 0;
      bit_index    <= 0;
      tx_shift     <= 10'b1111111111;
      uart_txd_in  <= 1;  // Idle state is high
      sending      <= 0;
      tx_done      <= 0;
    end else begin
      if (tx_start && !sending) begin
        // Load shift register with frame: start bit (0), 8 data bits (LSB first), stop bit (1)
        tx_shift <= {1'b1, tx_data, 1'b0};
        sending  <= 1;
        baud_counter <= 0;
        bit_index <= 0;
        tx_done <= 0;
      end else if (sending) begin
        if (baud_counter < BAUD_TICK - 1) begin
          baud_counter <= baud_counter + 1;
        end else begin
          baud_counter <= 0;
          uart_txd_in <= tx_shift[bit_index];
          if (bit_index < 9) begin
            bit_index <= bit_index + 1;
          end else begin
            sending   <= 0;
            tx_done   <= 1;
          end
        end
      end else begin
        tx_done <= 0;
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
module _chua_rng (
    input              sysclk,   // renamed from clk
    input              rst,      // synchronous reset (active high)
    input [31:0] C1_seed,
    input [31:0] C2_seed,
    input [31:0] L_seed,
    output reg [15:0]  rnd       // 16-bit random number output
);
  // State registers: Q16 fixed-point (32-bit signed)
  reg signed [31:0] v1;  // voltage across C1
  reg signed [31:0] v2;  // voltage across C2
  reg signed [31:0] iL;  // current through the inductor

  // Parameter definitions (in Q16 format)

  parameter signed [31:0] inv_C1 = 32'd6554;   // ~65536/10
  parameter signed [31:0] inv_C2 = 32'd655;    // ~65536/100
  parameter signed [31:0] inv_L  = 32'd3641;   // ~65536/18

  parameter signed [31:0] R  = 32'd10000;  // resistor value (Q16)
  parameter signed [31:0] dt = 32'd66;      // time step ≈0.001 in Q16 (66/65536)
  // Nonlinear resistor (Chua diode) parameters in Q16.
  parameter signed [31:0] Ga = -32'd49152; // approx. -0.75 in Q16
  parameter signed [31:0] Gb =  32'd32768; // approx. 0.5 in Q16
  parameter signed [31:0] B  =  32'd8192;  // approx. 0.125 in Q16

  // Nonlinear function for Chua's diode (piecewise-linear)
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
  reg signed [31:0] diff; // (v1 - v2)
  reg signed [31:0] term; // temporary term

  always @(posedge sysclk or posedge rst) begin
    if (rst) begin
      // Initialize state registers: seed values scaled to Q16.
      v1  <= C1_seed << 16;
      v2  <= C2_seed << 16;
      iL  <= L_seed  << 16;
      rnd <= 16'h0000;
    end else begin
      diff = v1 - v2;
      term = diff / R;
      dv1 = (inv_C1 * (term - nonlinear(v1))) >> 16;
      dv2 = (inv_C2 * ((-diff) / R + iL)) >> 16;
      diL = (inv_L * (-v2)) >> 16;

      // Euler integration update
      v1  <= v1  + ((dt * dv1) >> 16);
      v2  <= v2  + ((dt * dv2) >> 16);
      iL  <= iL  + ((dt * diL) >> 16);

      // Use lower 16 bits of v1 as random output
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
    input        sysclk,          // renamed from clk
    input        rst,
    output       uart_txd_in,     // matches constraint for UART TX output
    input        uart_rxd_out     // dummy port to satisfy board constraints; unused in this design
);
  // Wire from Chua RNG.
  wire [15:0] rnd;
  reg [15:0]  random_value;

  chua_rng rng_inst (
      .sysclk(sysclk),
      .rst(rst),
      .rnd(rnd)
  );

  // Signals for UART transmission.
  reg         tx_start;
  reg  [7:0]  tx_data;
  wire        tx_done;

  uart_tx uart_tx_inst (
      .sysclk(sysclk),
      .rst(rst),
      .tx_start(tx_start),
      .tx_data(tx_data),
      .uart_txd_in(uart_txd_in),
      .tx_done(tx_done)
  );

  // FSM to transmit the 16-bit random number in two sequential 8-bit segments.
  localparam STATE_IDLE       = 0;
  localparam STATE_SEND_HIGH  = 1;
  localparam STATE_WAIT_HIGH  = 2;
  localparam STATE_SEND_LOW   = 3;
  localparam STATE_WAIT_LOW   = 4;
  localparam STATE_DELAY      = 5;

  reg [2:0] state;
  reg [23:0] delay_counter; // Delay between transmissions.

  always @(posedge sysclk or posedge rst) begin
    if (rst) begin
      state         <= STATE_IDLE;
      tx_start      <= 0;
      tx_data       <= 8'd0;
      random_value  <= 16'd0;
      delay_counter <= 0;
    end else begin
      case (state)
        STATE_IDLE: begin
          // Latch current random value.
          random_value <= rnd;
          // Begin transmitting the high byte.
          tx_data  <= rnd[15:8];
          tx_start <= 1;
          state    <= STATE_SEND_HIGH;
        end
        STATE_SEND_HIGH: begin
          tx_start <= 0; // Clear start pulse.
          state    <= STATE_WAIT_HIGH;
        end
        STATE_WAIT_HIGH: begin
          if (tx_done) begin
            // High byte sent; now load low byte.
            tx_data  <= random_value[7:0];
            tx_start <= 1;
            state    <= STATE_SEND_LOW;
          end
        end
        STATE_SEND_LOW: begin
          tx_start <= 0;
          state    <= STATE_WAIT_LOW;
        end
        STATE_WAIT_LOW: begin
          if (tx_done) begin
            state         <= STATE_DELAY;
            delay_counter <= 0;
          end
        end
        STATE_DELAY: begin
          if (delay_counter < 24'd5000000) begin // Adjust delay as needed.
            delay_counter <= delay_counter + 1;
          end else begin
            state <= STATE_IDLE;
          end
        end
        default: state <= STATE_IDLE;
      endcase
    end
  end
endmodule
