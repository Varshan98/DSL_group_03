module top_module(
        input sysclk,
        input btn0,
        input btn1,
        //External ADC MCP3202 Pin;
        output adc_din,
        output adc_clk,
        output adc_csn,
        input  adc_dout,
        //UART Tx Pin;
        output uart_rxd_out,
        output pio38
);

//RESET SYSTEM CONFIG;
wire rstn;
reg rst;
assign rstn = ~btn0;
//if (btn1 == 1)begin
//assign rst = btn1;

assign pio38 = uart_rxd_out;
//CLOCK TREE CONFIG;
wire CLK500Hz,CLK1Hz,CLK_ADC,CLK_UART,CLK2Hz;

clock_div clk_div_u1(rstn,sysclk,CLK500Hz);
clock_div clk_div_u2(rstn,CLK500Hz,CLK1Hz);
clock_div clk_div_u3(rstn,sysclk,CLK_ADC);
clock_div clk_div_u4(rstn,sysclk,CLK_UART);
clock_div clk_div_u5(rstn,CLK500Hz,CLK2Hz);

always @(posedge CLK500Hz) begin
    rst <= btn1;
end

defparam clk_div_u1.FREQ_INPUT  = 12_000_000;
defparam clk_div_u1.FREQ_OUTPUT = 500;
defparam clk_div_u2.FREQ_INPUT  = 500;
defparam clk_div_u2.FREQ_OUTPUT = 16;
defparam clk_div_u3.FREQ_INPUT  = 12_000_000;
defparam clk_div_u3.FREQ_OUTPUT = 2_000_000;
defparam clk_div_u4.FREQ_INPUT  = 12_000_000;
defparam clk_div_u4.FREQ_OUTPUT = 9600;
defparam clk_div_u5.FREQ_INPUT  = 500;
defparam clk_div_u5.FREQ_OUTPUT = 32;


//7SEGMENT DISPLAY CONFIG;
reg adc_ready;
wire adc_vaild;
wire rnd_valid;
wire [11:0] adc_data; 
wire [15:0] Send_data;

        
//EXTERNAL ADC MCP3202 CONFIG;
// DRV FREQ : 2MHZ;
// CHANNEL : ONLY CHANNEL 0; 
localparam  SINGLE_CHAN0  = 2'b10;
localparam  SINGLE_CHAN1  = 2'b11;
reg [1:0] adc_ch;
reg [11:0] datach0,datach1;
reg [11:0] Segment_data;
wire data_received;
reg [15:0] Save_data;
 
drv_mcp3202 drv_mcp3202_u0(
    .rstn(rstn),
    .clk(CLK_ADC),
    .ap_ready(adc_ready),
    .ap_vaild(adc_vaild),  
    .mode(adc_ch), //yeekiat changed this
    .data(adc_data),

    .port_din(adc_dout),
    .port_dout(adc_din), //adc_din
    .port_clk(adc_clk),
    .port_cs(adc_csn)
);

// ADC SAMPLING EVENT (FREQ:1HZ)
always @(negedge rstn, posedge adc_vaild,posedge CLK_ADC) begin
    if(!rstn) begin
        adc_ready <= 1'b0;
        Segment_data <= 12'hABC;
    end else begin
        if(adc_vaild) begin
            Segment_data <= adc_data;
            adc_ready <= 1'b0;
        end
        else begin
            adc_ready <= 1'b1;
        end
    end
end

always @(posedge CLK_ADC)begin
 if (adc_ch == SINGLE_CHAN1)begin
    adc_ch <= SINGLE_CHAN0;
    datach1 <= Segment_data;
    end
 else begin
    adc_ch <= SINGLE_CHAN1;
    datach0 <= Segment_data;
    end
  end
  // SANITIZED SEED VALUES
  
wire [15:0] lfsr_out;

lfsr16 my_lfsr (
    .clk(CLK_UART),
    .rstn(rstn),
    .random(lfsr_out)
);

reg [16:0] ran_num;

always @(posedge sysclk) begin 
ran_num = lfsr_out;
end
wire [31:0] seed_sanitized = {ran_num[7:4],datach0[7:4],ran_num[3:0],datach0[3:0],ran_num[15:12],datach1[7:4],ran_num[13:8],datach1[3:0]};


chua_rng chua_rng_data(
    .clk(sysclk),
    .rstn(rstn),
    .start(rst),
    .seed(seed_sanitized),
    .done(data_received),
    .rand_val(Send_data)
); 
  
 always @(posedge CLK1Hz or negedge rstn) begin
    if(!rstn) begin
        Save_data <= 16'h123;
    end 
    else begin
        if (data_received == 1'd1) begin
            Save_data <= Send_data;
           end
          else begin
            Save_data <= 16'hFFFF;
          end
    end
end

//UART Tx Event Config (FREQ:9600)

reg uart_ready;
wire uart_vaild;
reg [7:0] uart_data;
reg hl_sel;

drv_uart_tx drv_uart_u0(
    .clk(CLK_UART),
    .ap_rstn(rstn),
    .ap_ready(uart_ready),
    .ap_vaild(uart_vaild),
    .tx(uart_rxd_out),
    .pairty(1'b0),
    .data(uart_data)
);

always @(negedge rstn, posedge uart_vaild, negedge CLK2Hz) begin
    if(!rstn) begin
        uart_ready <= 1'b0;
        hl_sel <= 1'b0;
    end else begin
        if(uart_vaild) begin
            uart_data <= (hl_sel)? Save_data[15:8]: Save_data[7:0];
            uart_ready   <= 1'b0;
        end
        else begin
            uart_ready  <= 1'b1;
            hl_sel      <= ~hl_sel;
        end
    end
end


endmodule        





//module debounce_rst (
//    input wire clk,          // system clock
//    input wire noisy_btn,    // raw button signal (active-low preferred)
//    output reg rst_pulse     // clean one-cycle-wide pulse
//);

//    // 2-stage synchronizer
//    reg sync_0 = 0, sync_1 = 0;
//    always @(posedge clk) begin
//        sync_0 <= ~noisy_btn;  // invert if button is active-low
//        sync_1 <= sync_0;
//    end

//    // Debounce counter
//    reg [15:0] debounce_cnt = 0;
//    reg debounced = 0;

//    parameter DEBOUNCE_THRESHOLD = 16'd50000; // adjust for clock speed

//    always @(posedge clk) begin
//        if (sync_1 != debounced) begin
//            debounce_cnt <= debounce_cnt + 1;
//            if (debounce_cnt > DEBOUNCE_THRESHOLD) begin
//                debounced <= sync_1;
//                debounce_cnt <= 0;
//            end
//        end else begin
//            debounce_cnt <= 0;
//        end
//    end

//    // Generate pulse on rising edge of debounced signal
//    reg debounced_prev = 0;
//    always @(posedge clk) begin
//        debounced_prev <= debounced;
//        rst_pulse <= (debounced && !debounced_prev);  // rising edge
//    end

//endmodule


    
//////////////////////////////////////////////////////////////////////////////////
// Engineer: Maoyang
// Create Date: 15.03.2024 15:41:25
// Module Name: clock_div
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//////////////////////////////////////////////////////////////////////////////////


module clock_div(
    input rstn,
    input clksrc,
    output clkout
);

parameter FREQ_INPUT  = 12_000_000;
parameter FREQ_OUTPUT = 1_000;
parameter CNTER_MAX = FREQ_INPUT/(FREQ_OUTPUT*2);
parameter CNTER_WIDTH = $clog2(CNTER_MAX);

reg clkout_r;
reg [CNTER_WIDTH-1:0] cnter;
assign clkout = clkout_r;

always @(negedge rstn,posedge clksrc) begin
    if(!rstn)begin
        cnter <= {CNTER_WIDTH{1'b0}};
        clkout_r <= 1'b0;
    end
    else begin
        if(cnter == CNTER_MAX - 1'b1)begin
            clkout_r <= ~clkout_r;
            cnter <= {CNTER_WIDTH{1'b0}};            
        end
        else begin
            cnter <= cnter + 1'b1;
        end
    end    
end

endmodule
        
/*
 * Module: drv_mcp3202
 * Date : 2024/03/21
 * Author : Maoyang
 * Description:
 * This Verilog module implements an SPI interface for the MCP3202 Analog-to-Digital Converter (ADC).
 * It manages the communication process through a finite state machine (FSM) to read analog data and convert it to a digital format.
 * 
 * Inputs:
 * - rstn: Active low reset signal.
 * - clk: Clock signal.
 * - ap_ready: Signal indicating the application is ready for data processing.
 * - mode: 2-bit input to select the ADC channel and configuration.
 *          localparam  SINGLE_CHAN0   = 2'b10; - CHANNEL 0;
 *          localparam  SINGLE_CHAN1   = 2'b11; - CHANNEL 1;
 *          localparam  DIFFER_CHAN01  = 2'b00;  - DIFFERENTIAL CHANNEL 01
 *          localparam  DIFFER_CHAN10  = 2'b01;  - DIFFERENTIAL CHANNEL 10
 * - port_din: Serial data input from the ADC.
 * 
 * Outputs:
 * - ap_vaild: Signal that indicates valid data is available.
 * - data: 12-bit output holding the ADC conversion result.
 * - port_dout: Serial data output to the ADC.
 * - port_clk: Clock signal for the ADC.
 * - port_cs: Chip select signal for the ADC, active low.
 * 
 * Functionality:
 * - The module configures the ADC based on the mode input.
 * - It uses an FSM to control the SPI communication process, including sending control bits, reading the ADC data, and signaling when new data is available.
 * - The ADC conversion result is made available as a 12-bit output.
 * - The module ensures synchronization with the external ADC device through careful management of clock and control signals.
 * 
 * Implementation Details:
 * - The Data_Transmit wire is used to send control signals to the ADC.
 * - The Data_Receive register captures the ADC output.
 * - The FSM transitions through several states (IDLE, WRITE, READ, STOP) to manage the entire data transfer process.
 * - The cnter_writ and cnter_read registers are used to track progress through the transmit and receive phases, respectively.
 */
 
module drv_mcp3202(
    input rstn,
    input clk,
    input   ap_ready,
    output  reg ap_vaild,
    input   [1:0] mode,
    output  [11:0] data,

    input   port_din,
    output  reg port_dout,
    output  port_clk,
    output  reg port_cs
);

wire    [3:0]      Data_Transmit; // 4 bits CONTROL;
reg     [12:0]     Data_Receive;  // 1 bit NULL + 12 bits DATA;

assign Data_Transmit[3]    = 1'b1;
assign Data_Transmit[0]    = 1'b1;
assign Data_Transmit[2:1] = mode;

reg [1:0]   fsm_statu,fsm_next;
localparam FSM_IDLE = 2'b00;
localparam FSM_WRIT = 2'b10;
localparam FSM_READ = 2'b11;
localparam FSM_STOP = 2'b01;

reg [1:0] cnter_writ;
reg [3:0] cnter_read;

//FSM statu transfer;
always @(posedge clk, negedge rstn) begin
    if (!rstn)  fsm_statu <= FSM_IDLE;
    else        fsm_statu <= fsm_next;
end

//FSM Transfer Condition;
always @(*)begin
    if(!rstn) fsm_next <= FSM_IDLE;
    else begin
        case (fsm_statu)
            FSM_IDLE : fsm_next <= (ap_ready)? FSM_WRIT : FSM_IDLE;
            FSM_WRIT : fsm_next <= (2'd0 == cnter_writ)? FSM_READ : FSM_WRIT;
            FSM_READ : fsm_next <= (2'd0 == cnter_read)? FSM_STOP : FSM_READ;
            FSM_STOP : fsm_next <= (!ap_ready)? FSM_STOP : FSM_IDLE;
            default  : fsm_next <= FSM_IDLE;
        endcase
    end
end

//FSM Output - SPI Write Data
always @(negedge rstn,negedge clk)begin
    if (!rstn) begin
        cnter_writ  <= 2'd3;
        port_dout   <= 1'b1;
        port_cs     <= 1'b1;
    end else begin
        case (fsm_statu)
            FSM_IDLE : begin 
                cnter_writ  <= 2'd3;
                port_dout   <= 1'b1;
                port_cs     <= 1'b1;
            end
            FSM_WRIT : begin 
                port_cs     <= 1'b0;
                port_dout   <= Data_Transmit[cnter_writ];
                cnter_writ  <= cnter_writ - 1'b1;
            end
            FSM_READ : begin 
                port_cs     <= 1'b0;
                port_dout   <= 1'b1;
            end
            FSM_STOP : port_cs     <= 1'b1;
            default  : ;
        endcase
    end
end

//FSM Output - SPI Read  Data
always @(negedge rstn,posedge clk)begin
    if (!rstn) begin
        cnter_read  <= 4'd13;
        Data_Receive <= 13'h00;
        ap_vaild = 1'b0;
    end else begin
        case (fsm_statu)
            FSM_IDLE : begin
                ap_vaild = 1'b0; 
                cnter_read  <= 4'd13;
            end
            FSM_WRIT : begin 
                Data_Receive <= 13'h00;
            end
            FSM_READ : begin 
                cnter_read <= cnter_read - 1'b1;
                Data_Receive[cnter_read] <= port_din;
            end
            FSM_STOP : ap_vaild = 1'b1;
            default  : ;
        endcase
    end
end

assign port_clk = clk | port_cs;
assign data = Data_Receive[11:0];

endmodule

// Module Name: uart_tx
// Date : 2024/03/24
// Version : V0.1
// Author : Maoyang
// Description: Implements a UART transmitter with optional parity bit functionality. This module encodes and transmits 
// data over a serial line using the UART protocol. It supports transmitting data with start, stop, and optional parity bits,
// and signals the completion of transmission.

// Inputs:
// clk: System clock signal for synchronizing the transmission process (Baud Rate).
// ap_rstn: Asynchronous, active low reset signal to initialize or reset the module's internal states.
// ap_ready: Input signal indicating readiness to start data transmission. When high, transmission begins.
// pairty: Input signal to enable (when high) or disable (when low) parity bit generation and transmission.
// data: 8-bit data input to be transmitted over UART.

// Outputs:
// ap_vaild: Output signal that indicates the complaetion of a data transmission cycle.
// tx: Serial output transmitting the encoded data along with start, stop, and optional parity bits.

// Local Parameters:
// FSM_IDLE, FSM_STAR, FSM_TRSF, FSM_PARI, FSM_STOP: Represent the states of the finite state machine (FSM) controlling
// the UART transmission process, from idle, through start bit, data transmission, optional parity bit, and stop bit.

// Internal Registers:
// fsm_statu: Holds the current state of the FSM.
// fsm_next: Determines the next state of the FSM based on the current state and input signals.
// cnter: Counter used during the data transmission state to index through the data bits.

// Behavioral Blocks:
// 1. fsm statu transfer: Sequential logic block that updates the current state of the FSM on each positive clock edge or
//    on negative edge of ap_rstn. Resets to FSM_IDLE on reset.
// 2. fsm conditional transfer: Combinatorial logic block that determines the next state of the FSM based on current 
//    conditions like ap_ready signal, counter value, and parity configuration.
// 3. fsm - output: Sequential logic block that performs actions based on the current FSM state, including setting the
//    tx output according to the data bits, generating a parity bit if enabled, and indicating the end of transmission 
//    through ap_vaild signal. Also handles the initialization of internal signals on reset.

// Note: This module is designed to be synthesized and integrated into larger systems requiring UART transmission 
// capabilities, with configurable support for parity bit for error detection.
module drv_uart_tx(
    input   clk,
    input   ap_rstn,
    input   ap_ready,
    output  reg ap_vaild,
    output  reg tx,
    input   pairty,
    input  [7:0] data
);

localparam  FSM_IDLE = 3'b000,
            FSM_STAR = 3'b001,
            FSM_TRSF = 3'b010,
            FSM_PARI = 3'b011,
            FSM_STOP = 3'b100;

reg [2:0] fsm_statu;
reg [2:0] fsm_next;
reg [2:0] cnter;

//fsm statu transfer;
always @(posedge clk, negedge ap_rstn) begin
    if (!ap_rstn)begin
        fsm_statu <= FSM_IDLE;
    end else begin
        fsm_statu <= fsm_next;
    end
end

//fsm conditional transfer;
always @(*)begin
    if(!ap_rstn)begin
        fsm_next <= FSM_IDLE;
    end else begin
        case(fsm_statu)
            FSM_IDLE:begin 
                fsm_next <= (ap_ready) ? FSM_STAR : FSM_IDLE;
            end
            FSM_STAR: fsm_next <= FSM_TRSF;
            FSM_TRSF:begin 
                fsm_next <= (cnter == 3'h7) ? (pairty?FSM_PARI:FSM_STOP) : FSM_TRSF;
            end
            FSM_PARI: fsm_next <= FSM_STOP;
            FSM_STOP:begin 
                fsm_next <= (!ap_ready) ? FSM_IDLE : FSM_STOP;
            end
            default: fsm_next <= FSM_IDLE;
        endcase
    end
end

//fsm - output
always @(posedge clk, negedge ap_rstn)begin
    if(!ap_rstn)begin
        ap_vaild <= 1'b0;
        tx <= 1'b1;
        cnter <= 3'h0;
    end else begin
        case (fsm_statu)
            FSM_IDLE: begin 
                tx <= 1'b1;
                ap_vaild <= 1'b0;
            end
            FSM_STAR: begin 
                tx <= 1'b0;
                cnter <= 3'h0;
            end
            FSM_TRSF: begin
                tx <= data[cnter];
                cnter <= cnter + 1'b1;
            end
            FSM_PARI: tx <= (^data); //Parity Check - ODD Check;
            FSM_STOP: begin
                tx <= 1'b1;         //Stop Bit;
                ap_vaild <= 1'b1;
            end
        endcase
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
    input wire clk,
    input wire rstn,
    input wire start,
    input wire [31:0] seed,
    output reg done,
    output reg [15:0] rand_val
);

    // --- Parameters ---
    reg [31:0] STEPS;

    // --- Fixed-point constants (Q16.16 format) ---
    reg signed [31:0] alpha = 32'd1027606;  // ~15.6 << 16
    reg signed [31:0] beta  = 32'd1835008;  // ~28.0 << 16
    reg signed [31:0] m0    = -32'd74865;   // ~-1.143 << 16
    reg signed [31:0] m1    = -32'd46721;   // ~-0.714 << 16

    // --- State ---
    reg signed [31:0] x, y, z;
    reg signed [31:0] dx, dy, dz;
    reg [15:0] step_count;
    reg running;

    // --- Chua f(x) ---
    function signed [31:0] chua_f;
        input signed [31:0] xin;
        begin
            if (xin < -32'd65536)  // -1 in Q16.16
                chua_f = m1 * xin + ((m0 - m1) <<< 15);
            else if (xin > 32'd65536) // +1 in Q16.16
                chua_f = m1 * xin - ((m0 - m1) <<< 15);
            else
                chua_f = m0 * xin;
        end
    endfunction

    // --- Main Logic ---
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            x <= 0;
            y <= 0;
            z <= 0;
            step_count <= 0;
            running <= 0;
            done <= 0;
            rand_val <= 0;
        end else begin
            if (start && !running) begin
                // Initialize with seed
                x <= {seed[15:0], 16'd0}; // Convert to Q16.16
                y <= {seed[31:16], 16'd0};
                z <= {seed[15:0] ^ seed[31:16], 16'd0};
                STEPS <= 16384 + (seed[7:0]<<4);
                step_count <= 0;
                running <= 1;
                done <= 0;
            end else if (running) begin
                dx = (alpha * (y - x - chua_f(x))) >>> 16;
                dy = (x - y + z);
                dz = (-beta * y) >>> 16;

                x <= x + (dx >>> 10);  // dt ~ 1/1024
                y <= y + (dy >>> 10);
                z <= z + (dz >>> 10);

                step_count <= step_count + 1;
                if (step_count == STEPS) begin
                    running <= 0;
                    done <= 1;
                    rand_val <= x[24:8];  // output top 16 bits
                end
            end
        end
    end

endmodule

module lfsr16 (
    input wire clk,
    input wire rstn,
    output reg [15:0] random
);

    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            random <= 16'hACE1;  // Initial non-zero seed (any non-zero value)
        else begin
            // Feedback taps for 16-bit maximal LFSR: x^16 + x^14 + x^13 + x^11 + 1
            random <= {random[14:0], random[15] ^ random[13] ^ random[12] ^ random[10]};
        end
    end

endmodule
