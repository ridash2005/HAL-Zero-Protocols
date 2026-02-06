/*
 * SystemVerilog I2C Master
 * 
 * Improvements:
 * - Converted to SystemVerilog.
 * - Implemented proper NACK error detection.
 * - Added `error_o` enum to specify type of error.
 */

typedef enum logic [1:0] {
    NO_ERROR,
    NACK_ADDR, // Slave didn't Ack Address
    NACK_DATA, // Slave didn't Ack Data
    BUS_ERROR  // Internal error
} i2c_error_t;

module i2c_master #(
    parameter int CLK_DIV = 250
)(
    input  logic       clk,
    input  logic       rst,
    input  logic       start_i,
    input  logic [6:0] slave_addr_i,
    input  logic [7:0] data_i,
    
    output logic       busy_o,
    output logic       done_o,
    output i2c_error_t error_o,
    
    // I2C Lines
    output logic i2c_scl_enable, 
    output logic i2c_sda_enable, 
    input  logic i2c_sda_in
);

    typedef enum logic [3:0] {
        IDLE,
        START_SEQ,
        ADDR,
        CHECK_ACK_ADDR,
        DATA_TX,
        CHECK_ACK_DATA,
        STOP_SEQ,
        ERROR_STATE
    } state_t;

    state_t state;
    logic [15:0] clk_cnt;
    logic [3:0]  bit_cnt;
    logic [7:0]  shift_reg;
    logic [7:0]  addr_rw;
    
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state           <= IDLE;
            i2c_scl_enable  <= 0;
            i2c_sda_enable  <= 0;
            busy_o          <= 0;
            done_o          <= 0;
            error_o         <= NO_ERROR;
            clk_cnt         <= 0;
            bit_cnt         <= 0;
        end else begin
            case (state)
                IDLE: begin
                    busy_o         <= 0;
                    i2c_scl_enable <= 0;
                    i2c_sda_enable <= 0;
                    
                    if (start_i) begin
                        state     <= START_SEQ;
                        busy_o    <= 1;
                        done_o    <= 0;
                        error_o   <= NO_ERROR;
                        addr_rw   <= {slave_addr_i, 1'b0}; // Write mode
                        shift_reg <= data_i;
                        clk_cnt   <= 0;
                    end
                end
                
                START_SEQ: begin
                    // SDA High->Low while SCL High
                    i2c_sda_enable <= 1; // Pull SDA Low
                    if (clk_cnt < CLK_DIV) clk_cnt <= clk_cnt + 1;
                    else begin
                        clk_cnt <= 0;
                        i2c_scl_enable <= 1; // Pull SCL Low
                        state <= ADDR;
                        bit_cnt <= 7;
                    end
                end
                
                ADDR: begin
                    // Shift out Address (7 bits) + Write Bit (0)
                    i2c_sda_enable <= ~addr_rw[bit_cnt];
                    if (clk_cnt < CLK_DIV - 1) begin
                        clk_cnt <= clk_cnt + 1;
                        if (clk_cnt == CLK_DIV/2) i2c_scl_enable <= 0; // SCL High (float)
                    end else begin
                        clk_cnt <= 0;
                        i2c_scl_enable <= 1; // SCL Low
                        if (bit_cnt == 0) begin
                            state <= CHECK_ACK_ADDR;
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end
                
                CHECK_ACK_ADDR: begin
                    i2c_sda_enable <= 0; // Release SDA for ACK
                    if (clk_cnt < CLK_DIV - 1) begin
                        clk_cnt <= clk_cnt + 1;
                        if (clk_cnt == CLK_DIV/4) i2c_scl_enable <= 0; // SCL High early
                        if (clk_cnt == 3*CLK_DIV/4) begin
                            // Sample SDA. If High, it's a NACK.
                            if (i2c_sda_in == 1) error_o <= NACK_ADDR;
                        end
                    end else begin
                        clk_cnt <= 0;
                        i2c_scl_enable <= 1; // SCL Low
                        if (error_o != NO_ERROR) state <= STOP_SEQ;
                        else begin
                            state <= DATA_TX;
                            bit_cnt <= 7;
                        end
                    end
                end

                DATA_TX: begin
                    i2c_sda_enable <= ~shift_reg[bit_cnt];
                    if (clk_cnt < CLK_DIV - 1) begin
                        clk_cnt <= clk_cnt + 1;
                        if (clk_cnt == CLK_DIV/2) i2c_scl_enable <= 0; // SCL High (float)
                    end else begin
                        clk_cnt <= 0;
                        i2c_scl_enable <= 1; // SCL Low
                        if (bit_cnt == 0) begin
                            state <= CHECK_ACK_DATA;
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                CHECK_ACK_DATA: begin
                    i2c_sda_enable <= 0; // Release SDA
                    if (clk_cnt < CLK_DIV - 1) begin
                        clk_cnt <= clk_cnt + 1;
                        if (clk_cnt == CLK_DIV/4) i2c_scl_enable <= 0; // SCL High
                        if (clk_cnt == 3*CLK_DIV/4) begin
                            if (i2c_sda_in == 1) error_o <= NACK_DATA;
                        end
                    end else begin
                        clk_cnt <= 0;
                        i2c_scl_enable <= 1; // SCL Low
                        state <= STOP_SEQ;
                    end
                end
                
                STOP_SEQ: begin
                    // Generate Stop Condition
                    i2c_scl_enable <= 0; // Float SCL High
                    i2c_sda_enable <= 0; // Float SDA High
                    state          <= IDLE;
                    
                    if (error_o == NO_ERROR) done_o <= 1;
                    busy_o <= 0;
                end
            endcase
        end
    end
endmodule
