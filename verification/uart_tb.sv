`timescale 1ns/1ps

module uart_tb;
    logic clk;
    logic rst;
    logic tx_start;
    logic [7:0] tx_data;
    logic tx_busy;
    logic tx_line;
    logic error;

    // Instantiate DUT
    uart_tx #(
        .CLK_FREQ(1000000), // 1MHz for simulation
        .BAUD_RATE(115200),
        .PARITY_ODD(0)
    ) dut (
        .clk(clk),
        .rst(rst),
        .tx_start_i(tx_start),
        .tx_data_i(tx_data),
        .tx_busy_o(tx_busy),
        .tx_line_o(tx_line),
        .error_o(error)
    );

    // Clock Generation
    initial begin
        clk = 0;
        forever #500 clk = ~clk; // 1MHz Clock
    end

    // Test sequence
    initial begin
        $dumpfile("uart_tb.vcd");
        $dumpvars(0, uart_tb);
        
        rst = 1;
        tx_start = 0;
        tx_data = 8'h00;
        #2000;
        rst = 0;
        #2000;

        // Sent 0xA5 (10100101)
        // Even Parity: ^(10100101) = 0. So parity bit = 0.
        // Data bits (LSB first): 1, 0, 1, 0, 0, 1, 0, 1
        tx_data = 8'hA5;
        tx_start = 1;
        #1000;
        tx_start = 0;

        wait(tx_busy == 0);
        #10000;
        
        $display("UART Test Complete");
        $finish;
    end

endmodule
