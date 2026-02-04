module uart_rx(
    input wire        clk,        // 50MHz clock
    input wire        rst_n,
    input wire        rx,
    output reg [7:0]  rx_msg,
    output reg        rx_complete
);

    // FSM states
    localparam IDLE            = 3'b000;
    localparam START_BIT       = 3'b001; 
    localparam DATA_BITS       = 3'b010;
    localparam STOP_BIT        = 3'b011;

    reg [2:0] state;
    reg [8:0] baud_counter;
    reg [2:0] bit_index;
    reg [7:0] shift_reg;
    reg       rx_sync1, rx_sync2;

    // ===== BAUD RATE CALCULATION =====
    // For 115200 baud at 50MHz:
    // Cycles per bit = 50,000,000 / 115,200 = 434.03 cycles
    // We'll use 434 cycles per bit
    // Mid-bit sample = 217 cycles (half of 434)
    localparam CYCLES_PER_BIT = 9'd434;
    localparam HALF_BIT       = 9'd217;

    // ===== INPUT SYNCHRONIZATION =====
    // Two-stage synchronizer to prevent metastability
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= rx;
            rx_sync2 <= rx_sync1;
        end
    end

    // ===== MAIN FSM =====
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            baud_counter <= 9'd0;
            bit_index    <= 3'd0;
            shift_reg    <= 8'd0;
            rx_msg       <= 8'd0;
            rx_complete  <= 1'b0;
        end else begin
            
            rx_complete <= 1'b0;  // Default: clear the complete flag
            
            case (state)
                // ===== IDLE STATE =====
                // Wait for start bit (falling edge: 1 → 0)
                IDLE: begin
                    baud_counter <= 9'd0;
                    bit_index    <= 3'd0;
                    
                    if (rx_sync2 == 1'b0) begin  // Start bit detected
                        state <= START_BIT;
                    end
                end

                // ===== START BIT =====
                // Sample at mid-bit to confirm valid start bit
                START_BIT: begin
                    if (baud_counter == HALF_BIT) begin
                        if (rx_sync2 == 1'b0) begin
                            // Valid start bit confirmed
                            baud_counter <= 9'd0;
                            state        <= DATA_BITS;
                        end else begin
                            // False start - return to idle
                            state <= IDLE;
                        end
                    end else begin
                        baud_counter <= baud_counter + 1'b1;
                    end
                end

                // ===== DATA BITS (8 bits, LSB first) =====
                DATA_BITS: begin
                    if (baud_counter == CYCLES_PER_BIT - 1) begin
                        // Sample the bit
                        shift_reg    <= {rx_sync2, shift_reg[7:1]};
                        baud_counter <= 9'd0;
                        
                        if (bit_index == 3'd7) begin
                            // All 8 bits received
                            state <= STOP_BIT;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                        end
                    end else begin
                        baud_counter <= baud_counter + 1'b1;
                    end
                end

                // ===== STOP BIT =====
                STOP_BIT: begin
                    if (baud_counter == CYCLES_PER_BIT - 1) begin
                        if (rx_sync2 == 1'b1) begin
                            // Valid stop bit - output the received byte
                            rx_msg      <= shift_reg;
                            rx_complete <= 1'b1;
                        end
                        // Return to idle regardless (error or success)
                        state <= IDLE;
                    end else begin
                        baud_counter <= baud_counter + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule