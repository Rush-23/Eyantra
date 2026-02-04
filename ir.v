/*
# Team ID:          1024
# Theme:            MazeSolver Bot
# Author List:      Rushil V, Indiran T, Sathiya Naarayanan C, Charan Karthick A S
# Filename:         ir
# File Description: ir module 
# Global variables: N/A
*/

module ir(
    input  wire clk,
    input  wire rst_n,
    input  wire ir_in,          // Digital IR sensor (1 = tall wall detected)

    output reg  deadend_pulse   // 1-clock pulse on rising IR
);

    reg ir_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ir_prev       <= 1'b0;
            deadend_pulse <= 1'b0;
        end else begin
            deadend_pulse <= 1'b0;        // default

            if (!ir_in) begin  // rising edge detected
                deadend_pulse <= 1'b1;
            end

            ir_prev <= ir_in;
        end
    end
endmodule