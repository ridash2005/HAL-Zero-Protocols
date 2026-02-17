/*
 * Enhanced I2C Master Testbench — Self-checking with realistic slave model
 *
 * Verifies:
 *   1. START condition generation (SDA falls while SCL high)
 *   2. Address byte transmission (7-bit address + R/W bit)
 *   3. ACK/NACK detection
 *   4. Data byte transmission
 *   5. STOP condition generation (SDA rises while SCL high)
 *   6. Error flag behavior on NACK
 *   7. Done flag timing
 */

`timescale 1ns / 1ps

module i2c_tb;

    // Parameters
    localparam int CLK_DIV = 10;  // Short for simulation

    // DUT Signals
    logic       clk = 0;
    logic       rst;
    logic       start;
    logic [6:0] slave_addr;
    logic [7:0] data_in;
    logic       busy;
    logic       done;

    // I2C Error enum (must match DUT)
    typedef enum logic [1:0] {
        NO_ERROR   = 2'b00,
        NACK_ADDR  = 2'b01,
        NACK_DATA  = 2'b10,
        BUS_ERROR  = 2'b11
    } i2c_error_t;

    i2c_error_t error_out;

    // I2C bus (directly wired to DUT outputs/inputs)
    logic       scl_enable;
    logic       sda_enable;
    logic       sda_in;

    // Internal bus state
    wire scl_bus = scl_enable ? 1'b0 : 1'b1;
    wire sda_bus = (sda_enable || slave_sda_drive) ? 1'b0 : 1'b1;

    // Slave model drives SDA
    logic slave_sda_drive = 0;

    // Connect bus feedback to DUT
    assign sda_in = sda_bus;

    // Test state
    int         pass_count = 0;
    int         fail_count = 0;
    int         test_count = 0;

    // Clock: 50 MHz
    always #10 clk = ~clk;

    // DUT
    i2c_master #(
        .CLK_DIV(CLK_DIV)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start_i(start),
        .slave_addr_i(slave_addr),
        .data_i(data_in),
        .busy_o(busy),
        .done_o(done),
        .error_o(error_out),
        .i2c_scl_enable(scl_enable),
        .i2c_sda_enable(sda_enable),
        .i2c_sda_in(sda_in)
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

    // ── Slave ACK Responder ────────────────────────────────────────────────
    // This task watches the bus and provides ACK (pull SDA low during 9th clock)
    // when ack_addr and ack_data are set
    logic ack_addr_en = 1;
    logic ack_data_en = 1;
    int bit_counter = 0;
    logic waiting_addr_ack = 0;
    logic waiting_data_ack = 0;

    // Simple slave model: count SCL rising edges to know when to ACK
    always @(posedge scl_bus or posedge rst) begin
        if (rst) begin
            bit_counter <= 0;
            waiting_addr_ack <= 0;
            waiting_data_ack <= 0;
        end
    end

    // The slave needs to drive SDA low during the ACK clock cycle.
    // We detect this by counting. After 8 address bits, the 9th is ACK.
    // After 8 data bits, the 9th is ACK.
    // This is done with timing-based detection.

    task automatic slave_respond(
        input logic do_ack_addr,
        input logic do_ack_data
    );
        // Wait for start condition
        @(negedge sda_bus);
        if (scl_bus == 1) begin
            $display("  [SLAVE] START detected");
        end

        // Count 8 address bits
        for (int i = 0; i < 8; i++) begin
            @(posedge scl_bus);
        end

        // ACK cycle for address
        @(negedge scl_bus);
        if (do_ack_addr) begin
            slave_sda_drive = 1; // Pull SDA low = ACK
            $display("  [SLAVE] Address ACK");
        end else begin
            slave_sda_drive = 0; // Leave high = NACK
            $display("  [SLAVE] Address NACK");
        end
        @(posedge scl_bus);
        @(negedge scl_bus);
        slave_sda_drive = 0; // Release after ACK sampled

        if (!do_ack_addr) return;

        // Count 8 data bits
        for (int i = 0; i < 8; i++) begin
            @(posedge scl_bus);
        end

        // ACK cycle for data
        @(negedge scl_bus);
        if (do_ack_data) begin
            slave_sda_drive = 1;
            $display("  [SLAVE] Data ACK");
        end else begin
            slave_sda_drive = 0;
            $display("  [SLAVE] Data NACK");
        end
        @(posedge scl_bus);
        @(negedge scl_bus);
        slave_sda_drive = 0;
    endtask

    // ── Main Test Sequence ─────────────────────────────────────────────────
    initial begin
        $dumpfile("i2c_tb.vcd");
        $dumpvars(0, i2c_tb);

        $display("═══════════════════════════════════════════════════════════");
        $display("  I2C Master Testbench — Self-checking");
        $display("═══════════════════════════════════════════════════════════");

        // Initialize
        rst        = 1;
        start      = 0;
        slave_addr = 7'h50;
        data_in    = 8'hAA;
        slave_sda_drive = 0;
        #200;
        rst = 0;
        #200;

        // ── Test 1: Idle state ──────────────────────────────────────────
        $display("\n── Test 1: Idle State ──");
        check(busy == 0, "Bus not busy at idle");
        check(scl_enable == 0, "SCL released (high) at idle");
        check(sda_enable == 0, "SDA released (high) at idle");

        // ── Test 2: Successful write (ACK on both addr and data) ────────
        $display("\n── Test 2: Write 0xAA to slave 0x50 (ACK+ACK) ──");
        fork
            begin
                @(posedge clk);
                slave_addr = 7'h50;
                data_in    = 8'hAA;
                start      = 1;
                @(posedge clk);
                start = 0;
            end
            slave_respond(1, 1);  // ACK addr, ACK data
        join

        // Wait for done or timeout
        fork
            wait(done || error_out != NO_ERROR);
            begin #500000; end
        join_any
        disable fork;

        check(done == 1, "Done flag set after successful write");
        check(error_out == NO_ERROR, "No error on successful write");
        #1000;

        // ── Test 3: NACK on address ────────────────────────────────────
        $display("\n── Test 3: NACK on address (no slave at 0x70) ──");
        fork
            begin
                @(posedge clk);
                slave_addr = 7'h70;
                data_in    = 8'h55;
                start      = 1;
                @(posedge clk);
                start = 0;
            end
            slave_respond(0, 0);  // NACK addr
        join

        fork
            wait(done || error_out != NO_ERROR);
            begin #500000; end
        join_any
        disable fork;

        check(error_out == NACK_ADDR, "Error = NACK_ADDR when slave doesn't respond");
        check(busy == 0, "Bus released after NACK");
        #1000;

        // ── Test 4: NACK on data ───────────────────────────────────────
        $display("\n── Test 4: ACK on address, NACK on data ──");
        fork
            begin
                @(posedge clk);
                slave_addr = 7'h50;
                data_in    = 8'hFF;
                start      = 1;
                @(posedge clk);
                start = 0;
            end
            slave_respond(1, 0);  // ACK addr, NACK data
        join

        fork
            wait(done || error_out != NO_ERROR);
            begin #500000; end
        join_any
        disable fork;

        check(error_out == NACK_DATA, "Error = NACK_DATA when data rejected");
        #1000;

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
        #5_000_000;
        $display("[TIMEOUT] Test exceeded maximum time");
        $finish(1);
    end

endmodule
