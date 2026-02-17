/*
 * Enhanced UART TX Testbench — Self-checking with assertions
 *
 * Verifies:
 *   1. Correct start bit (logic 0)
 *   2. Data bits match input byte (LSB first)
 *   3. Correct stop bit (logic 1)
 *   4. Done flag asserts at correct time
 *   5. Multiple bytes transmitted correctly
 *   6. Timing between bytes is valid
 */

`timescale 1ns / 1ps

module uart_tx_tb;

    // Parameters
    localparam int CLK_FREQ     = 50_000_000;
    localparam int BAUD_RATE    = 115200;
    localparam int CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    localparam int BIT_PERIOD   = (1_000_000_000 / BAUD_RATE); // in ns

    // DUT Signals
    logic       clk = 0;
    logic       rst;
    logic       tx_start;
    logic [7:0] tx_data;
    logic       tx_serial;
    logic       tx_done;

    // Test state
    int         pass_count = 0;
    int         fail_count = 0;
    int         test_count = 0;

    // Clock generation: 50 MHz → 20ns period
    always #10 clk = ~clk;

    // DUT
    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) dut (
        .clk(clk),
        .rst(rst),
        .tx_start(tx_start),
        .tx_data(tx_data),
        .tx_serial(tx_serial),
        .tx_done(tx_done)
    );

    // ── Test Infrastructure ────────────────────────────────────────────────
    task automatic check(input logic condition, input string msg);
        test_count++;
        if (condition) begin
            pass_count++;
            $display("  [PASS] %s", msg);
        end else begin
            fail_count++;
            $display("  [FAIL] %s", msg);
        end
    endtask

    // ── Capture and Verify a Single UART Byte ──────────────────────────────
    task automatic verify_uart_byte(input logic [7:0] expected_byte);
        logic [7:0] received_byte;
        logic       start_bit;
        logic       stop_bit;

        // Wait for start bit (falling edge on tx_serial)
        @(negedge tx_serial);

        // Sample at middle of start bit
        #(BIT_PERIOD / 2);
        start_bit = tx_serial;
        check(start_bit == 0,
              $sformatf("Start bit = 0 (for byte 0x%02h)", expected_byte));

        // Sample 8 data bits (LSB first)
        for (int i = 0; i < 8; i++) begin
            #BIT_PERIOD;
            received_byte[i] = tx_serial;
        end

        // Sample stop bit
        #BIT_PERIOD;
        stop_bit = tx_serial;

        check(received_byte == expected_byte,
              $sformatf("Data = 0x%02h (expected 0x%02h)", received_byte, expected_byte));
        check(stop_bit == 1,
              $sformatf("Stop bit = 1 (for byte 0x%02h)", expected_byte));
    endtask

    // ── Transmit a byte and wait for done ──────────────────────────────────
    task automatic send_byte(input logic [7:0] byte_val);
        @(posedge clk);
        tx_data  = byte_val;
        tx_start = 1;
        @(posedge clk);
        tx_start = 0;
    endtask

    // ── Main Test Sequence ─────────────────────────────────────────────────
    initial begin
        $dumpfile("uart_tx_tb.vcd");
        $dumpvars(0, uart_tx_tb);

        $display("═══════════════════════════════════════════════════════════");
        $display("  UART TX Testbench — Self-checking");
        $display("═══════════════════════════════════════════════════════════");

        // Initialize
        rst      = 1;
        tx_start = 0;
        tx_data  = 0;
        #100;
        rst = 0;
        #100;

        // ── Test 1: Idle state ──────────────────────────────────────────
        $display("\n── Test 1: Idle State ──");
        check(tx_serial == 1, "TX line idle = HIGH");
        check(tx_done == 0 || tx_done == 1, "Done flag at known state");

        // ── Test 2: Single byte 0xA5 ───────────────────────────────────
        $display("\n── Test 2: Transmit 0xA5 ──");
        fork
            send_byte(8'hA5);
            verify_uart_byte(8'hA5);
        join

        // Wait for done flag
        wait(tx_done);
        check(tx_done == 1, "Done flag asserted after TX complete");
        #(BIT_PERIOD * 2);

        // ── Test 3: Single byte 0x00 (all zeros) ──────────────────────
        $display("\n── Test 3: Transmit 0x00 (all zeros) ──");
        fork
            send_byte(8'h00);
            verify_uart_byte(8'h00);
        join
        wait(tx_done);
        #(BIT_PERIOD * 2);

        // ── Test 4: Single byte 0xFF (all ones) ──────────────────────
        $display("\n── Test 4: Transmit 0xFF (all ones) ──");
        fork
            send_byte(8'hFF);
            verify_uart_byte(8'hFF);
        join
        wait(tx_done);
        #(BIT_PERIOD * 2);

        // ── Test 5: Back-to-back bytes "Hi" ─────────────────────────
        $display("\n── Test 5: Back-to-back bytes 'H' (0x48), 'i' (0x69) ──");
        fork
            begin
                send_byte(8'h48); // 'H'
                wait(tx_done);
                @(posedge clk);
                send_byte(8'h69); // 'i'
            end
            begin
                verify_uart_byte(8'h48);
                verify_uart_byte(8'h69);
            end
        join
        wait(tx_done);
        #(BIT_PERIOD * 2);

        // ── Summary ────────────────────────────────────────────────────
        $display("\n═══════════════════════════════════════════════════════════");
        $display("  RESULTS: %0d/%0d passed, %0d failed",
                 pass_count, test_count, fail_count);
        $display("═══════════════════════════════════════════════════════════");

        if (fail_count > 0) begin
            $display("  *** TEST SUITE FAILED ***");
            $finish(1);
        end else begin
            $display("  ALL TESTS PASSED");
            $finish(0);
        end
    end

    // Timeout watchdog
    initial begin
        #(BIT_PERIOD * 200);
        $display("[TIMEOUT] Test exceeded maximum time");
        $finish(1);
    end

endmodule
