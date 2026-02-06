`timescale 1ns/1ps

module spi_tb;
    logic clk;
    logic rst;
    logic start;
    logic [7:0] data_i;
    logic [7:0] data_o;
    logic done;
    logic error;
    
    logic sck;
    logic mosi;
    logic miso;
    logic cs_n;

    // Instantiate DUT
    spi_master #(
        .CLK_DIV(2)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start_i(start),
        .data_i(data_i),
        .data_o(data_o),
        .done_o(done),
        .error_o(error),
        .sck_o(sck),
        .mosi_o(mosi),
        .miso_i(miso),
        .cs_n_o(cs_n)
    );

    // Clock Generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz Clock
    end

    // Slave Model (Simple Loopback)
    // SPI Mode 0: Data must be available when CS goes low
    assign miso = cs_n ? 1'b0 : slave_data[bit_idx];
    
    logic [7:0] slave_data = 8'h5A;
    integer bit_idx;

    always @(negedge sck or posedge cs_n) begin
        if (cs_n) begin
            bit_idx <= 7;
        end else begin
            if (bit_idx > 0) bit_idx <= bit_idx - 1;
            else bit_idx <= 7;
        end
    end

    // Test sequence
    initial begin
        $dumpfile("spi_tb.vcd");
        $dumpvars(0, spi_tb);
        
        rst = 1;
        start = 0;
        data_i = 8'h00;
        #20;
        rst = 0;
        #20;

        data_i = 8'hA5;
        start = 1;
        #10;
        start = 0;

        wait(done == 1);
        #50;
        
        if (data_o == 8'h5A) $display("SPI Test Passed: Received 0x5A");
        else $display("SPI Test Failed: Received %h", data_o);
        
        $finish;
    end

endmodule
