`default_nettype none

module tt_um_uart_receiver (
    input  wire clk,      // clock
    input  wire rst_n,    // reset_n - low to reset
    input  wire ena,      // enable signal (active high)
    input  wire rx,       // UART receive line

    // Outputs
    output reg [6:0] data_out, // Received Hamming(7,4) data output (7 bits)
    output reg [1:0] state_out, // Current state of the receiver
    output reg valid_out  // Indicates if the received data is valid
);

    // -------------------------------------------------------------------------- //
    // State encoding for receiver
    localparam [1:0] RX_IDLE  = 2'b00,
                     RX_START = 2'b01,
                     RX_DATA  = 2'b10,
                     RX_STOP  = 2'b11;

    // -------------------------------------------------------------------------- //
    // State and control registers
    reg [1:0] state;          // Current state
    reg [2:0] bit_counter;    // Counts data bits (0-6 for 7 Hamming bits)
    reg [2:0] sample_counter; // Oversampling counter

    assign state_out = state; // Output current state for debugging

    // -------------------------------------------------------------------------- //
    // Main state machine logic

    always @(posedge clk or negedge rst_n) begin    
        if (!rst_n) begin
            // Reset logic
            state <= RX_IDLE;
            bit_counter <= 3'b000;
            sample_counter <= 3'b000;
            data_out <= 7'b0000000;
            valid_out <= 1'b0;
        end else if (ena) begin
            
            case (state)
                // RX_IDLE: Wait for start bit (rx goes HIGH in inverted UART)
                RX_IDLE: begin
                    if (rx == 1'b0) begin  // Start bit detected (LOW in inverted UART)
                        state <= RX_START;
                        sample_counter <= 3'b001;
                    end
                end
                
                // RX_START: Sample middle of start bit
                RX_START: begin
                    // Oversample start bit, change state only at end
                    if (sample_counter == 3'b111) begin
                        if (rx == 1'b0) begin       // Start bit is LOW
                            state <= RX_DATA;
                            bit_counter <= 3'b000;
                            sample_counter <= 3'b000;
                            data_out <= 7'b0000000;     // Reset data register when entering START state
                            valid_out <= 1'b0;          // Reset valid output
                        end else begin // Invalid start bit
                            state <= RX_IDLE;
                            sample_counter <= 3'b000;
                        end
                    end else begin
                        sample_counter <= sample_counter + 1;
                    end
                    
                end
                
                // RX_DATA: Receive 7 data bits for Hamming(7,4) code
                RX_DATA: begin
                    if (sample_counter == 3'b011) begin
                        data_out <= {rx, data_out[6:1]}; // LSB first
                        sample_counter <= sample_counter + 1;

                    end else if (sample_counter == 3'b111) begin
                        sample_counter <= 3'b000; // Reset counter for next bit

                        // check if all bits received
                        if (bit_counter == 3'b110) begin
                            // All 7 bits received (bit 0 through bit 6)
                            state <= RX_STOP;
                            bit_counter <= 3'b000; // Reset bit counter for next frame
                        end else begin
                            bit_counter <= bit_counter + 1;
                        end
                    end else begin
                        sample_counter <= sample_counter + 1;
                    end
                end
                
                // RX_STOP: Check for stop bit (should be HIGH in UART)
                RX_STOP: begin
                    if (sample_counter == 3'b111) begin
                        state <= RX_IDLE;
                        sample_counter <= 3'b000;
                    end else if(sample_counter == 3'b011) begin
                        valid_out <= rx; // Stop bit is HIGH
                        sample_counter <= sample_counter + 1;
                    end else begin
                        sample_counter <= sample_counter + 1;
                    end
                end
                
                default: state <= RX_IDLE;
            endcase
        end
    end

endmodule