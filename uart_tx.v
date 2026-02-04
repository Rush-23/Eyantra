module uart_tx(
    input clk, rst,
    input parity_type,
    input tx_start,
    input [7:0] data,
    output reg tx, tx_done,
    output wire busy
);

parameter IDLE = 3'b000, START = 3'b001, DATA = 3'b010,
          PARITY = 3'b011, STOP = 3'b100, DONE = 3'b101;

reg [16:0] baud_count = 0;
reg [2:0]  current = IDLE, next = IDLE;
reg [2:0]  bit_index;
reg [7:0]  data_buffer;

// ================================
// State & counters (SEQUENTIAL)
// ================================
always @(posedge clk or negedge rst) begin
    if (!rst) begin
        current     <= IDLE;
        baud_count  <= 0;
        bit_index   <= 0;
        data_buffer <= 0;
    end else begin
        current <= next;

        // Reset baud_count when entering a new state
        if (current != next)
            baud_count <= 0;
        else
            baud_count <= baud_count + 1;

        // Latch data and reset index in IDLE
        if (current == IDLE && tx_start) begin
            data_buffer <= data;
            bit_index   <= 0;
        end

        // Increment bit_index every time one bit period (434 cycles) passes
        if (current == DATA) begin
            if (baud_count == 433) begin
                baud_count <= 0; // Manual reset for baud_count within DATA state
                bit_index  <= bit_index + 1;
            end
        end
    end
end

// ================================
// Next-state logic (COMBINATIONAL)
// ================================
always @(*) begin
    case (current)
        IDLE:   next = tx_start ? START : IDLE;
        START:  next = (baud_count == 433) ? DATA : START;
        // Move to PARITY after 8 bits (bit_index reaches 7 and the 434th cycle of that bit finishes)
        DATA:   next = (bit_index == 3'd7 && baud_count == 433) ? PARITY : DATA;
        PARITY: next = (baud_count == 433) ? STOP : PARITY;
        STOP:   next = (baud_count == 433) ? DONE : STOP;
        DONE:   next = IDLE;
        default:next = IDLE;
    endcase
end

// ================================
// Output logic (COMBINATIONAL)
// ================================
always @(*) begin
    tx      = 1'b1;
    tx_done = 1'b0;

    case (current)
        IDLE:   tx = 1'b1;
        START:  tx = 1'b0;
        DATA:   tx = data_buffer[bit_index];
        // Parity: ^data_buffer is Even. XOR with parity_type toggles it.
        PARITY: tx = ^data_buffer ^ parity_type;
        STOP:   tx = 1'b1;
        DONE:   begin 
            tx = 1'b1;
            tx_done = 1'b1;
        end
    endcase
end
assign busy = (current != IDLE);
endmodule