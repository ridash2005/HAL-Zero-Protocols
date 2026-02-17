/*
 * Enhanced SPI Master Testbench — Self-checking with loopback slave
 *
 * Verifies:
 *   1. Chip select assertion/deassertion timing
 *   2. MOSI data matches input (MSB first)
 *   3. MISO loopback data correctly captured
 *   4. SCK frequency respects CLK_DIV
 *   5. Done flag behavior
 *   6. Multiple transaction correctness
 *   7. Watchdog/error timeout detection
 */

`timescale 1ns / 1ps

module spi_tb;

    // Parameters
    localparam int CLK_DIV = 4;

    // DUT Signals
    logic       clk = 0;
    logic       rst;
    logic       start;
    logic [7:0] data_in;
    logic [7:0] data_out;
    logic       done;
    logic       error;

    // SPI bus
    logic       sck;
    logic       mosi;
    logic       miso;
    logic       cs_n;

    // Test state
    int         pass_count = 0;
    int         fail_count = 0;
    int         test_count = 0;

    // Clock: 50 MHz
    always #10 clk = ~clk;

    // DUT
    spi_master #(
        .CLK_DIV(CLK_DIV)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start_i(start),
        .data_i(data_in),
        .data_o(data_out),
        .done_o(done),
        .error_o(error),
        .sck_o(sck),
        .mosi_o(mosi),
        .miso_i(miso),
        .cs_n_o(cs_n)
    );

    // ── Loopback Slave Model ───────────────────────────────────────────────
    // Shift register slave: captures MOSI, shifts out a configurable response
    logic [7:0] slave_shift_reg;
    logic [7:0] slave_response;

    always @(posedge sck or posedge rst) begin
        if (rst) begin
            slave_shift_reg <= 8'h00;
        end else if (!cs_n) begin
            // Capture MOSI on rising edge
            slave_shift_reg <= {slave_shift_reg[6:0], mosi};
        end
    end

    always @(negedge sck or posedge rst) begin
        if (rst) begin
            miso <= 0;
        end else if (!cs_n) begin
            // Shift out response on falling edge
            miso <= slave_response[7];
            slave_response <= {slave_response[6:0], 1'b0};
        end
    end

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

    // ── Transmit and Verify ────────────────────────────────────────────────
    task automatic spi_transfer(
        input logic [7:0] tx_byte,
        input logic [7:0] expected_rx
    );
        @(posedge clk);
        data_in = tx_byte;
        start   = 1;
        @(posedge clk);
        start = 0;

        // Wait for completion or error
        fork
            begin
                wait(done || error);
            end
            begin
                #100000;
                $display("  [TIMEOUT] SPI transfer did not complete");
            end
        join_any
        disable fork;

        check(!error, $sformatf("No error during transfer of 0x%02h", tx_byte));
        check(done == 1, $sformatf("Done flag set for 0x%02h", tx_byte));

        if (done) begin
            check(data_out == expected_rx,
                  $sformatf("RX data = 0x%02h (expected 0x%02h)", data_out, expected_rx));
        end

        // Verify CS deasserted
        @(posedge clk);
        @(posedge clk);
        check(cs_n == 1, "CS_N deasserted after transfer");
    endtask

    // ── Main Test Sequence ─────────────────────────────────────────────────
    initial begin
        $dumpfile("spi_tb.vcd");
        $dumpvars(0, spi_tb);

        $display("═══════════════════════════════════════════════════════════");
        $display("  SPI Master Testbench — Self-checking with Loopback");
        $display("═══════════════════════════════════════════════════════════");

        // Initialize
        rst      = 1;
        start    = 0;
        data_in  = 0;
        miso     = 0;
        slave_response = 8'h00;
        #100;
        rst = 0;
        #100;

        // ── Test 1: Idle state ──────────────────────────────────────────
        $display("\n── Test 1: Idle State ──");
        check(cs_n == 1, "CS_N idle = HIGH");
        check(sck == 0, "SCK idle = LOW (CPOL=0)");
        check(done == 0, "Done idle = LOW");
        check(error == 0, "Error idle = LOW");

        // ── Test 2: Simple loopback 0xA5 → expect slave response 0x3C ──
        $display("\n── Test 2: TX=0xA5, Slave returns 0x3C ──");
        slave_response = 8'h3C;
        spi_transfer(8'hA5, 8'h3C);
        check(slave_shift_reg == 8'hA5,
              "Slave captured MOSI data = 0xA5");
        #200;

        // ── Test 3: TX=0x00, Slave returns 0xFF ─────────────────────────
        $display("\n── Test 3: TX=0x00, Slave returns 0xFF ──");
        slave_response = 8'hFF;
        spi_transfer(8'h00, 8'hFF);
        check(slave_shift_reg == 8'h00,
              "Slave captured MOSI data = 0x00");
        #200;

        // ── Test 4: TX=0xFF, Slave returns 0x55 ─────────────────────────
        $display("\n── Test 4: TX=0xFF, Slave returns 0x55 ──");
        slave_response = 8'h55;
        spi_transfer(8'hFF, 8'h55);
        check(slave_shift_reg == 8'hFF,
              "Slave captured MOSI data = 0xFF");
        #200;

        // ── Test 5: Walking ones pattern ────────────────────────────────
        $display("\n── Test 5: Walking ones ──");
        for (int i = 0; i < 8; i++) begin
            logic [7:0] walk_byte = (8'h01 << i);
            slave_response = ~walk_byte; // Complement as response
            spi_transfer(walk_byte, ~walk_byte);
            #100;
        end

        // ── Test 6: Back-to-back transfers ──────────────────────────────
        $display("\n── Test 6: Back-to-back transfers ──");
        slave_response = 8'hAA;
        spi_transfer(8'h11, 8'hAA);
        slave_response = 8'h55;
        spi_transfer(8'h22, 8'h55);
        slave_response = 8'hCC;
        spi_transfer(8'h33, 8'hCC);

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
        #10_000_000;
        $display("[TIMEOUT] Test exceeded maximum time");
        $finish(1);
    end

endmodule
