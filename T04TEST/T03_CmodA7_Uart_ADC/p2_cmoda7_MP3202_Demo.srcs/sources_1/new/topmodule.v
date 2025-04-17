module top_module(
    input sysclk,
    input [1:0] btn,
    output [1:0] led,
    output adc_din,
    output adc_clk,
    output adc_csn,
    input  adc_dout,
    output uart_rxd_out
);

// RESET SYSTEM CONFIG
wire rstn;
assign rstn = ~btn[1];

// CLOCK TREE CONFIG
wire CLK500Hz, CLK1Hz, CLK_ADC, CLK_UART, CLK2Hz;

clock_div clk_div_u1(rstn, sysclk, CLK500Hz);
clock_div clk_div_u2(rstn, CLK500Hz, CLK1Hz);
clock_div clk_div_u3(rstn, sysclk, CLK_ADC);
clock_div clk_div_u4(rstn, sysclk, CLK_UART);
clock_div clk_div_u5(rstn, sysclk, CLK2Hz);

defparam clk_div_u1.FREQ_INPUT  = 12_000_000;
defparam clk_div_u1.FREQ_OUTPUT = 500;
defparam clk_div_u2.FREQ_INPUT  = 500;
defparam clk_div_u2.FREQ_OUTPUT = 1;
defparam clk_div_u3.FREQ_INPUT  = 12_000_000;
defparam clk_div_u3.FREQ_OUTPUT = 2_000_000;
defparam clk_div_u4.FREQ_INPUT  = 12_000_000;
defparam clk_div_u4.FREQ_OUTPUT = 9600;
defparam clk_div_u5.FREQ_INPUT  = 12_000_000;
defparam clk_div_u5.FREQ_OUTPUT = 2; // 2Hz update rate for UART

// ADC CONFIG
localparam SINGLE_CHAN0 = 2'b10;

wire [11:0] adc_data;
reg [11:0] adc_ch0_data;
reg adc_ready;
wire adc_vaild;

drv_mcp3202 drv_mcp3202_u0(
    .rstn(rstn),
    .clk(CLK_ADC),
    .ap_ready(adc_ready),
    .ap_vaild(adc_vaild),
    .mode(SINGLE_CHAN0),
    .data(adc_data),
    .port_din(adc_dout),
    .port_dout(adc_din),
    .port_clk(adc_clk),
    .port_cs(adc_csn)
);

// ADC SAMPLING
always @(posedge CLK1Hz or negedge rstn or posedge adc_vaild) begin
    if (!rstn) begin
        adc_ready <= 1'b0;
        adc_ch0_data <= 12'd0;
    end else begin
        if (adc_vaild) begin
            adc_ch0_data <= adc_data;
            adc_ready <= 1'b0;
        end else begin
            adc_ready <= 1'b1;
        end
    end
end

// LED LOGIC
assign led[0] = (adc_ch0_data < 12'd800);  // LED ON when sound is loud
reg blink;
always @(posedge adc_vaild) begin
    blink <= ~blink;
end
assign led[1] = blink; // Blink on each new ADC sample

// UART CONFIG
reg uart_ready;
wire uart_vaild;
reg [7:0] uart_data;

drv_uart_tx drv_uart_u0(
    .clk(CLK_UART),
    .ap_rstn(rstn),
    .ap_ready(uart_ready),
    .ap_vaild(uart_vaild),
    .tx(uart_rxd_out),
    .pairty(1'b0),
    .data(uart_data)
);

// UART SEND ADC_CH0 DATA (8-bit scaled)
always @(posedge CLK2Hz or negedge rstn) begin
    if (!rstn) begin
        uart_ready <= 1'b0;
        uart_data <= 8'd0;
    end else begin
        uart_data <= adc_ch0_data[11:4];  // Send top 8 bits
        uart_ready <= 1'b1;
    end
end

endmodule
