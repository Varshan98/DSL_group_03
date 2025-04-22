`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2025 22:42:07
// Design Name: 
// Module Name: drv_mcp3202
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

module top_module (
    input        clk,            // 50 MHz system clock
    input        rst_btn,        // Pushbutton (active high)
    input        adc_miso,       // MCP3202 MISO (port_din)
    output       adc_mosi,       // MCP3202 MOSI (port_dout)
    output       adc_clk,        // MCP3202 SCLK (port_clk)
    output       adc_cs,         // MCP3202 CS
    output       uart_tx         // UART TX line
);

    // Internal signals
    wire [11:0] adc_data;
    reg         adc_ready = 1'b1; // For testing, constantly request samples
    wire        adc_valid;
    wire        rst_debounced;

    // Debounce reset button
    debounce rst_debounce_inst (
        .clk(clk),
        .noisy(rst_btn),
        .clean(rst_debounced)
    );

    // ADC driver for MCP3202
    drv_mcp3202 adc_driver (
        .rstn(~rst_debounced),
        .clk(clk),
        .ap_ready(adc_ready),
        .ap_vaild(adc_valid),
        .mode(2'b00),             // CH0 single-ended
        .data(adc_data),
        .port_din(adc_miso),
        .port_dout(adc_mosi),
        .port_clk(adc_clk),
        .port_cs(adc_cs)
    );

    // Chua RNG + UART logic
    chua_uart_rng rng_uart_inst (
        .clk(clk),
        .rst(rst_debounced),
        .uart_rxd_out(uart_tx),
        .dummy_uart_rx(1'b1) // Not used
    );

endmodule


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
