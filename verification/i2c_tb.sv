`timescale 1ns/1ps

module i2c_tb;
    logic clk;
    logic rst;
    logic start;
    logic [6:0] slave_addr;
    logic [7:0] data_i;
    logic busy;
    logic done;
    
    // I2C Errors (Must match i2c_master.sv)
    typedef enum logic [1:0] {
        NO_ERROR,
        NACK_ADDR,
        NACK_DATA,
        BUS_ERROR
    } i2c_error_t;
    
    i2c_error_t error_o;
    
    logic scl_enable;
    logic sda_enable;
    logic sda_in;
    reg slave_sda_out;

    tri1 scl, sda;
    assign scl = scl_enable ? 1'b0 : 1'bz;
    assign sda = sda_enable ? 1'b0 : 1'bz;
    assign sda = slave_sda_out ? 1'b0 : 1'bz;
    assign sda_in = sda;

    i2c_master #(
        .CLK_DIV(10)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start_i(start),
        .slave_addr_i(slave_addr),
        .data_i(data_i),
        .busy_o(busy),
        .done_o(done),
        .error_o(error_o),
        .i2c_scl_enable(scl_enable),
        .i2c_sda_enable(sda_enable),
        .i2c_sda_in(sda_in)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk; 
    end

    // Slave Model: ACK whenever Master releases SDA during transaction
    assign sda = (busy && sda_enable == 0) ? 1'b0 : 1'bz;

    initial begin
        $dumpfile("i2c_tb.vcd");
        $dumpvars(0, i2c_tb);
        
        rst = 1;
        start = 0;
        slave_addr = 7'h50;
        data_i = 8'hA5;
        slave_sda_out = 0;
        #20;
        rst = 0;
        #20;

        start = 1; #10; start = 0;

        // Wait for completion
        fork
            begin
                wait(done || error_o != NO_ERROR);
            end
            begin
                #50000; // Increased timeout
                $display("I2C Test Timeout at %t", $time);
            end
        join_any

        #100;
        if (done && error_o == NO_ERROR) $display("I2C Test Passed: Transaction Complete");
        else $display("I2C Test Failed: done=%b, error=%d at %t", done, error_o, $time);
        
        $finish;
    end

endmodule
