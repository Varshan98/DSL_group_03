`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03.04.2025 18:43:59
// Design Name: 
// Module Name: PRNG
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


// -----------------------------------------------------------------------------
// uart_tx.v
// A simple UART transmitter module.
// Transmits 10-bit UART frames: 1 start bit (0), 8 data bits (LSB first), 1 stop bit (1).
// Baud rate is determined from CLOCK_FREQ and BAUD_RATE parameters.
// -----------------------------------------------------------------------------
module uart_tx (
    input        clk,
    input        rst,
    input        tx_start,         // pulse to start transmission
    input  [7:0] tx_data,          // data to transmit
    output reg   tx,               // UART TX line
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

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      baud_counter <= 0;
      bit_index    <= 0;
      tx_shift     <= 10'b1111111111;
      tx           <= 1;
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
          tx <= tx_shift[bit_index];
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

// -----------------------------------------------------------------------------
// chua_rng.v
// Digital simulation of Chua's circuit using Euler integration.
// Generates a 16-bit random number by extracting the lower 16 bits of v1.
// Fixed-point format: Q16 (32-bit signed; 16 integer and 16 fractional bits)
// The seed parameters (C1_seed, C2_seed, L_seed) are used for initialization.
// -----------------------------------------------------------------------------
module chua_rng (
    input              clk,
    input              rst,    // synchronous reset (active high)
    output reg [15:0]  rnd     // 16-bit random number output
);
  // State registers: Q16 fixed-point (32-bit signed)
  reg signed [31:0] v1;  // voltage across C1
  reg signed [31:0] v2;  // voltage across C2
  reg signed [31:0] iL;  // current through the inductor

  //--------------------------------------------------------------------------
  // Parameter definitions (Q16 format where needed)
  //--------------------------------------------------------------------------
  // Seed parameters (interpreted as integer values, then scaled to Q16)
  parameter signed [31:0] C1_seed = 10;   // e.g., 10 nF
  parameter signed [31:0] C2_seed = 100;  // e.g., 100 nF
  parameter signed [31:0] L_seed  = 18;   // e.g., 18 mH

  // Use seeds as simulation parameters (their inverses in Q16)
  // (For example: inv_C1 = 65536 / C1_seed)
  parameter signed [31:0] inv_C1 = 32'd6554;   // ~65536/10
  parameter signed [31:0] inv_C2 = 32'd655;    // ~65536/100
  parameter signed [31:0] inv_L  = 32'd3641;   // ~65536/18

  // Resistor R and integration time step dt in Q16.
  parameter signed [31:0] R  = 32'd10000;  // arbitrary resistor value in Q16
  parameter signed [31:0] dt = 32'd66;      // dt ≈ 0.001 (66/65536) in Q16
  // Nonlinear resistor (Chua diode) parameters in Q16.
  // Ga: inner slope (typically negative), Gb: outer slope, B: breakpoint.
  parameter signed [31:0] Ga = -32'd49152; // approx. -0.75 in Q16
  parameter signed [31:0] Gb =  32'd32768; // approx. 0.5 in Q16
  parameter signed [31:0] B  =  32'd8192;  // approx. 0.125 in Q16

  //--------------------------------------------------------------------------
  // Nonlinear function implementation (Chua diode)
  //--------------------------------------------------------------------------
  // Implements a simple piecewise-linear function:
  //   if (v1 > B) or (v1 < -B) use outer slope Gb; else use inner slope Ga.
  function signed [31:0] nonlinear;
    input signed [31:0] x;
    begin
      if (x > B)
        nonlinear = (Gb * x) >>> 16; // Q16 multiplication: (Gb*x)/2^16
      else if (x < -B)
        nonlinear = (Gb * x) >>> 16;
      else
        nonlinear = (Ga * x) >>> 16;
    end
  endfunction

  //--------------------------------------------------------------------------
  // Internal signals for derivative calculations (all in Q16)
  //--------------------------------------------------------------------------
  reg signed [31:0] dv1, dv2, diL;
  reg signed [31:0] diff;   // (v1 - v2)
  reg signed [31:0] term;   // temporary term

  //--------------------------------------------------------------------------
  // Main integration block (Euler method)
  //--------------------------------------------------------------------------
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      // Initialize state registers.
      // Scale the seeds to Q16 by shifting left 16 bits.
      v1  <= C1_seed <<< 16;
      v2  <= C2_seed <<< 16;
      iL  <= L_seed  <<< 16;
      rnd <= 16'd0;
    end else begin
      diff = v1 - v2;

      // dv1/dt = inv_C1 * ((v1-v2)/R - nonlinear(v1))
      term = diff / R;  
      dv1 = (inv_C1 * (term - nonlinear(v1))) >>> 16;

      // dv2/dt = inv_C2 * ( - (v1-v2)/R + iL )
      dv2 = (inv_C2 * ((-diff) / R + iL)) >>> 16;

      // diL/dt = inv_L * (-v2)
      diL = (inv_L * (-v2)) >>> 16;

      // Euler integration update: new_state = old_state + dt * derivative
      v1  <= v1  + ((dt * dv1) >>> 16);
      v2  <= v2  + ((dt * dv2) >>> 16);
      iL  <= iL  + ((dt * diL) >>> 16);

      // Extract 16-bit random number (using the lower 16 bits of v1)
      rnd <= v1[15:0];
    end
  end
endmodule

// -----------------------------------------------------------------------------
// chua_uart_rng.v
// Top-level module that instantiates the Chua RNG and a UART transmitter.
// It latches a new 16-bit random number from the Chua simulation and sends it
// out via UART as two sequential 8-bit bytes (high byte first, then low byte).
// -----------------------------------------------------------------------------
module chua_uart_rng (
    input  clk,
    input  rst,
    output tx
);
  // Wire from Chua RNG.
  wire [15:0] rnd;
  reg [15:0]  random_value;

  chua_rng rng_inst (
      .clk(clk),
      .rst(rst),
      .rnd(rnd)
  );

  // UART transmitter instance signals.
  reg         tx_start;
  reg  [7:0]  tx_data;
  wire        tx_done;

  uart_tx uart_tx_inst (
      .clk(clk),
      .rst(rst),
      .tx_start(tx_start),
      .tx_data(tx_data),
      .tx(tx),
      .tx_done(tx_done)
  );

  // State machine to send the 16-bit random number in two bytes.
  localparam STATE_IDLE       = 0;
  localparam STATE_SEND_HIGH  = 1;
  localparam STATE_WAIT_HIGH  = 2;
  localparam STATE_SEND_LOW   = 3;
  localparam STATE_WAIT_LOW   = 4;
  localparam STATE_DELAY      = 5;

  reg [2:0] state;
  reg [23:0] delay_counter; // Delay between transmissions.
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      state         <= STATE_IDLE;
      tx_start      <= 0;
      tx_data       <= 8'd0;
      random_value  <= 16'd0;
      delay_counter <= 0;
    end else begin
      case (state)
        STATE_IDLE: begin
          // Latch the current random value.
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