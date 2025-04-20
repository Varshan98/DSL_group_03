/*
 * Module: top_module
 * Date : 2024/03/21
 * Author : Maoyang
 * Description:
 * This top-level module integrates various components to demonstrate a complete system design,
 * including a clock divider, a 7-segment display driver, and an external ADC (Analog-to-Digital Converter) MCP3202 interface.
 * The system operates based on the input from a system clock (sysclk) and two buttons (btn).
 * 
 * Inputs:
 * - sysclk: The system clock input.
 * - btn: 2-button input for system control. btn[1] is used as a reset signal.
 * - adc_dout: Digital output from the MCP3202 ADC.
 * 
 * Outputs:
 * - pio37 to pio48: Pins connected to a 7-segment display for displaying ADC data.
 * - led: 2-bit LED output for indicating system status or errors (not implemented in the given code).
 * - adc_din: Serial data input to the ADC.
 * - adc_clk: Clock signal for the ADC.
 * - adc_csn: Chip select signal for the ADC, active low.
 * 
 * Functionality:
 * 1. The module starts with configuring the system reset based on the btn[1] input.
 * 2. It then sets up three clock dividers to generate different frequencies needed for the 7-segment display and the ADC.
 * 3. The 7-segment display is driven by the drv_segment module, which updates the display based on the ADC data.
 * 4. The MCP3202 ADC is configured to operate in single channel mode (CHANNEL 0), with a 2MHz clock.
 * 5. ADC sampling occurs at a frequency of 1Hz, triggered by the CLK1Hz signal. When new ADC data is valid (adc_vaild), it is displayed on the 7-segment display.
 * 
 * Implementation Details:
 * - Clock division is achieved using instances of the clock_div module, parameterized to produce the required frequencies for the ADC and the 7-segment display.
 * - The 7-segment display configuration and updating are handled by the drv_segment module.
 * - The drv_mcp3202 module interfaces with the MCP3202 ADC, handling the SPI communication and data conversion process.
 * - The ADC sampling and display update logic is implemented in an always block, which reacts to the ADC's valid data signal and the 1Hz clock signal.
 * 
 * Notes:
 * - This module demonstrates handling of digital and analog inputs, clock management, and driving peripheral devices in a hardware description language.
 */
module top_module(
        input sysclk,
        input [1:0] btn,
        output [1:0] led,
        //External ADC MCP3202 Pin;
        output adc_din,
        output adc_clk,
        output adc_csn,
        input  adc_dout,
        //UART Tx Pin;
        output uart_rxd_out
);

//RESET SYSTEM CONFIG;
wire rstn;
assign rstn = ~btn[1];

//CLOCK TREE CONFIG;
wire CLK500Hz,CLK1Hz,CLK_ADC,CLK_UART,CLK2Hz;

clock_div clk_div_u1(rstn,sysclk,CLK500Hz);
clock_div clk_div_u2(rstn,CLK500Hz,CLK1Hz);
clock_div clk_div_u3(rstn,sysclk,CLK_ADC);
clock_div clk_div_u4(rstn,sysclk,CLK_UART);
clock_div clk_div_u5(rstn,sysclk,CLK2Hz);

defparam clk_div_u1.FREQ_INPUT  = 12_000_000;
defparam clk_div_u1.FREQ_OUTPUT = 500;
defparam clk_div_u2.FREQ_INPUT  = 500;
defparam clk_div_u2.FREQ_OUTPUT = 1;
defparam clk_div_u3.FREQ_INPUT  = 12_000_000;
defparam clk_div_u3.FREQ_OUTPUT = 2_000_000;
defparam clk_div_u4.FREQ_INPUT  = 12_000_000;
defparam clk_div_u4.FREQ_OUTPUT = 9600;
defparam clk_div_u5.FREQ_INPUT  = 12_000_000;
defparam clk_div_u5.FREQ_OUTPUT = 2;

//7SEGMENT DISPLAY CONFIG;
reg [11:0] Segment_data;
       
//EXTERNAL ADC MCP3202 CONFIG;
// DRV FREQ : 2MHZ;
// CHANNEL : ONLY CHANNEL 0; 
localparam  SINGLE_CHAN0  = 2'b10;
localparam  SINGLE_CHAN1  = 2'b11;

reg adc_ready;
wire adc_vaild;
wire [11:0] adc_data;

drv_mcp3202 drv_mcp3202_u0(
    .rstn(rstn),
    .clk(CLK_ADC),
    .ap_ready(adc_ready),
    .ap_vaild(adc_vaild),  
    .mode(SINGLE_CHAN0),
    .data(adc_data),

    .port_din(adc_dout),
    .port_dout(adc_din), //adc_din
    .port_clk(adc_clk),
    .port_cs(adc_csn)
);

// ADC SAMPLING EVENT (FREQ:1HZ)
always @(negedge rstn, posedge adc_vaild,posedge CLK1Hz) begin
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


//UART Tx Event Config (FREQ:9600)

reg uart_ready;
wire uart_vaild;
reg [7:0] uart_data;
reg hl_sel;
// added-start
reg [3:0] uart_state;
reg [13:0] adc_val;
reg [3:0] digit_idx;
reg [7:0] ascii_digits[0:5]; // 4 digits + \r + \n
// added-end
drv_uart_tx drv_uart_u0(
    .clk(CLK_UART),
    .ap_rstn(rstn),
    .ap_ready(uart_ready),
    .ap_vaild(uart_vaild),
    .tx(uart_rxd_out),
    .pairty(1'b0),
    .data(uart_data)
);

// replaced-start
always @(negedge rstn or posedge CLK2Hz) begin
    if (!rstn) begin
        uart_state <= 0;
        uart_ready <= 0;
        digit_idx <= 0;
    end else begin
        case (uart_state)
            0: begin
                adc_val <= Segment_data;
                // Convert to ASCII digits (manual divmod)
                ascii_digits[0] <= ((Segment_data / 1000) % 10) + 8'd48;
                ascii_digits[1] <= ((Segment_data / 100) % 10) + 8'd48;
                ascii_digits[2] <= ((Segment_data / 10) % 10) + 8'd48;
                ascii_digits[3] <= (Segment_data % 10) + 8'd48;
                ascii_digits[4] <= 8'd13; // '\r'
                ascii_digits[5] <= 8'd10; // '\n'
                digit_idx <= 0;
                uart_state <= 1;
            end
            1: begin
                uart_data <= ascii_digits[digit_idx];
                uart_ready <= 1;
                uart_state <= 2;
            end
            2: begin
                if (uart_vaild) begin
                    uart_ready <= 0;
                    digit_idx <= digit_idx + 1;
                    if (digit_idx == 5)
                        uart_state <= 0;
                    else
                        uart_state <= 1;
                end
            end
        endcase
    end
end
// replaced-end

endmodule