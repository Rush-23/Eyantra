/*
# Team ID:          1024
# Theme:            MazeSolver Bot
# Author List:      Rushil V, Indiran T, Sathiya Naarayanan C, Charan Karthick A S
# Filename:         encoder
# File Description: Encoder module to count the revolutions of the motor and calculate tick value
# Global variables: N/A
*/

module encoder (
    input  wire clk,
    input  wire reset,

    input  wire enc_a,
    input  wire enc_b,

    output reg signed [31:0] ticks
);

    // -----------------------------
    // Synchronizers
    // -----------------------------
    reg [1:0] a_sync, b_sync;

    always @(posedge clk) begin
        a_sync <= {a_sync[0], enc_a};
        b_sync <= {b_sync[0], enc_b};
    end

    wire a = a_sync[1];
    wire b = b_sync[1];

    // -----------------------------
    // Quadrature decode
    // -----------------------------
    reg [1:0] prev_state;
    wire [1:0] curr_state = {a, b};

    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            ticks      <= 0;
            prev_state <= 2'b00;
        end else begin
            case ({prev_state, curr_state})
                // forward
                4'b0001, 4'b0111, 4'b1110, 4'b1000:
                    ticks <= ticks + 1;

                // reverse
                4'b0010, 4'b0100, 4'b1101, 4'b1011:
                    ticks <= ticks - 1;

                default:
                    ticks <= ticks;
            endcase

            prev_state <= curr_state;
        end
    end

endmodule

