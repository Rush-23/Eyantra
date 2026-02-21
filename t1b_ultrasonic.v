/*
# Team ID:          1024
# Theme:            MazeSolver Bot
# Author List:      Rushil V, Indiran T, Sathiya Naarayanan C, Charan Karthick A S
# Filename:         t1b_ultrasonic
# File Description: Ultrasonic module code
# Global variables: N/A
*/

module t1b_ultrasonic(
    input clk_50M, 
    input reset, 
    input echo_rx,
    input enable,
    output reg trig,
    output op,
    output wire [15:0] distance_out
);

    // ==========================================
    // Internal Constants
    // ==========================================
    // 50MHz Clock = 20ns per tick
    localparam [2:0] 
        IDLE            = 3'b000,
        TRIGGER         = 3'b001,
        WAIT_ECHO_RISE  = 3'b010,
        MEASURE_ECHO    = 3'b011,
        CALC_DIST       = 3'b100,
        WAIT_DELAY      = 3'b101;

    // Timing Values (based on 50MHz / 20ns clock)
    // 10 us trigger  = 500 ticks
    // 12 ms delay    = 600,000 ticks
    // Timeout (30ms) = 1,500,000 ticks (Safety to prevent hanging)
    localparam [21:0] TRIG_WIDTH = 22'd500;
    localparam [21:0] WAIT_TIME  = 22'd2000000; 
    localparam [21:0] TIMEOUT    = 22'd1500000;

    // ==========================================
    // Registers
    // ==========================================
    reg [2:0] current_state, next_state;
    reg [21:0] counter = 0;      // Main timer
    reg [21:0] echo_width;   // Stores how long Echo was high
    reg [15:0] dist_reg;     // Internal register for distance
    reg op_reg;              // Internal register for OP
    reg trig_sent;

    reg echo_ff1, echo_ff2;

    

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            echo_ff1 <= 1'b0;
            echo_ff2 <= 1'b0;
        end else begin
            echo_ff1 <= echo_rx;
            echo_ff2 <= echo_ff1;
        end
    end
    wire echo_sync = echo_ff2;


    // ==========================================
    // State Machine Update & Counter
    // ==========================================
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            current_state <= IDLE;
            counter <= 0;
        end 
        /* else if (!enable) begin
            current_state <= IDLE;
            counter <= 0;
        end */
        else begin
            // If state changes, reset counter automatically
            if (current_state != next_state) begin
                current_state <= next_state;
                counter <= 0; 
            end else begin
                // Otherwise, keep counting
                counter <= counter + 1;
            end
        end
    end

    // ==========================================
    // Next State Logic
    // ==========================================
    always @(*) begin
        next_state = current_state; // Default: Stay in current state

        case (current_state)
            IDLE: begin
                if(enable) next_state = TRIGGER;
                else next_state = IDLE;

            end

            TRIGGER: begin
                // Hold Trig High for 10us
                if (counter >= TRIG_WIDTH) 
                    next_state = WAIT_ECHO_RISE;
            end

            WAIT_ECHO_RISE: begin
                // Wait for Echo to go High (Start of measurement)
                if (echo_sync && trig_sent) 
                    next_state = MEASURE_ECHO;
                // Safety: If no echo after 30ms, abort to prevent hang
                else if (counter >= TIMEOUT) 
                    next_state = WAIT_DELAY; 
            end

            MEASURE_ECHO: begin
                // Wait for Echo to go Low (End of measurement)
                if (echo_sync == 1'b0) 
                    next_state = CALC_DIST;
                // Safety: If echo stuck high > 30ms, abort
                else if (counter >= TIMEOUT) 
                    next_state = WAIT_DELAY;
            end

            CALC_DIST: begin
                // Single cycle state to update outputs
                next_state = WAIT_DELAY;
            end

            WAIT_DELAY: begin
                // Wait 12ms before next measurement
                if (counter >= WAIT_TIME) 
                    next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end

    // ==========================================
    // Output Logic
    // ==========================================
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            trig <= 0;
            dist_reg <= 0;
            op_reg <= 0;
            echo_width <= 0;
            trig_sent <= 0;
        end 
		  else if (!enable) begin
            trig <= 0;
            op_reg <= 0;
            trig_sent <= 0;
        end else begin
            case (current_state)
                TRIGGER: begin
                    trig <= 1'b1;
                    trig_sent <= 1'b1;
                end

                WAIT_ECHO_RISE: begin
                    trig <= 1'b0;
                end

                MEASURE_ECHO: begin
                    // Store the counter value continuously while Echo is High
                    // Since counter auto-resets when entering this state, 
                    // 'counter' equals the pulse width exactly.
                    echo_width <= counter;
                    trig_sent <= 1'b0;
                end

                CALC_DIST: begin
                    // Calculation: 
                    // Dist_mm = (counter * 0.00343)
                    // Approximation: (counter * 225) / 65536
                    dist_reg <= (echo_width * 230) >> 16;
                    
                    // Object Present Logic (Threshold < 70mm)
                    // Added check > 0 to ensure 0mm (timeout/error) isn't counted as an object
                    if ( ((echo_width * 230) >> 16) < 225 && ((echo_width * 230) >> 16) > 0 )
                        op_reg <= 1'b1; 
                    else if(((echo_width * 230) >> 16) > 225)
                        op_reg <= 1'b0;
                   // done <= 1'b1;
                end
            endcase
        end
    end

    // ==========================================
    // Output Assignments
    // ==========================================
    assign distance_out = dist_reg;
    assign op = op_reg;

endmodule