/*
# Team ID:          1024
# Theme:            MazeSolver Bot
# Author List:      Rushil V, Indiran T, Sathiya Naarayanan C, Charan Karthick A S
# Filename:         controller
# File Description: controller module to send commands to the motor
# Global variables: N/A
*/

module controller (
    input  wire clk,
    input  wire reset,

    input  wire [2:0] move,          // from maze solver

    input  wire encL_A,
    input  wire encL_B,
    input  wire encR_A,
    input  wire encR_B,

    input  wire  [15:0] dist1,dist2,dist3,

    output reg  [2:0] to_motordriver, // to motor driver
    output reg        enable,
    output reg        move_done,
	input  wire maze_ack,
    output reg  mpi_start,  // signal to mpicontroller to start
    input  wire ir,
	input  wire mpi_done,
    input wire maze_done,
    output reg  uturn_done,
    output reg signed [16:0] deltaL_reg, deltaR_reg
);

    // =====================================================
    // Encoder instances
    // =====================================================
    wire signed [31:0] left_ticks;
    wire signed [31:0] right_ticks;

    encoder ENC_LEFT (
        .clk(clk), .reset(reset),
        .enc_a(encL_A), .enc_b(encL_B),
        .ticks(left_ticks)
    );

    encoder ENC_RIGHT (
        .clk(clk), .reset(reset),
        .enc_a(encR_A), .enc_b(encR_B),
        .ticks(right_ticks)
    );
    
    // =====================================================
    // Commands
    // =====================================================
    localparam STOP        = 3'b000,
               FORWARD     = 3'b001,
               LEFT        = 3'b010,
               RIGHT       = 3'b011,
               UTURN       = 3'b100,
               UTURN_NMPI  = 3'b101,
               DRIFT_RIGHT = 3'b110,
               DRIFT_LEFT  = 3'b111;

    

    

    // =====================================================
    // FSM states
    // =====================================================
    localparam IDLE   = 3'b000;
    localparam MOVING = 3'b001;
    localparam POST_FORWARD = 3'b010;
	localparam DONE   = 3'b011;
	localparam WAIT   = 3'b111;
    localparam POST_FORWARD_UTURN  = 3'b101;
    localparam UTURN_STATE = 3'b110;

    reg [2:0] state, next_state;

    // =====================================================
    // Properly latched move
    // =====================================================
    reg [2:0] current_move;
    reg [2:0] move_prev;
    reg stopped;
    reg [2:0] mpi_count;
    

    // =====================================================
    // Tick bookkeeping
    // =====================================================
    reg signed [31:0] start_left;
    reg signed [31:0] start_right;

    // Use $signed() to force signed arithmetic
    wire signed [31:0] delta_left  = $signed(left_ticks)  - $signed(start_left);
    wire signed [31:0] delta_right = $signed(right_ticks) - $signed(start_right);

    wire [31:0]  abs_left  = (delta_left < 0 ? -delta_left : delta_left);
    wire [31:0]  abs_right = (delta_right < 0 ? -delta_right : delta_right);

    wire [31:0]  avg_turn = (abs_left + abs_right) >> 1;

    // =====================================================
    // Calibration parameters
    // =====================================================
    localparam FWD_TICKS = 32'd4950, LTICK_90 = 32'd1400, RTICK_90 =32'd1400, TICK_180 = 32'd2950, POST_FWD_TICKS = 32'd5290;

    // =====================================================
    // WAIT timing (1 second)
    // =====================================================
    parameter CLK_FREQ    = 50_000_000;  // 20ns per cycle 
    parameter WAIT_CYCLES = CLK_FREQ;
    parameter PAUSE_CYCLES = 1_000_000; // 2s delay
    reg [25:0] wait_counter;
    reg [32:0] post_for_counter;
    reg [31:0] uturn_counter = 32'd0;
    reg [2:0] steer_cmd;

    // =====================================================
    // State register
    // =====================================================
    always @(posedge clk or negedge reset) begin
        if (!reset)
            state <= IDLE;
        else
            state <= next_state;
    end

            

    // =====================================================
    // Track previous move (EDGE DETECTION)
    // =====================================================
    always @(posedge clk or negedge reset) begin
        if (!reset)
            move_prev <= STOP;
        else
            move_prev <= move;
    end

    // =====================================================
    // Latch move ONLY on STOP → MOVE transition
    // =====================================================
    always @(posedge clk or negedge reset) begin
    if (!reset) begin
        current_move <= STOP;
        start_left   <= 0;
        start_right  <= 0;
    end
    else if ((state == IDLE && move != STOP))begin
        current_move <= move;        
        start_left   <= left_ticks;
        start_right  <= right_ticks;
    end
    else if (state == MOVING && next_state == POST_FORWARD) begin
        start_left <= left_ticks;
        start_right <= right_ticks;
    end

    else if (state == WAIT && wait_counter >= WAIT_CYCLES) begin
        current_move <= STOP;          // release after move done
    end
end


    // =====================================================
    // WAIT counter
    // =====================================================
    always @(posedge clk or negedge reset) begin
        if (!reset)
            wait_counter <= 0;
        else if (state == WAIT) begin
            if (wait_counter < WAIT_CYCLES)
                wait_counter <= wait_counter + 1;
        end else
            wait_counter <= 0;
    end

    always @(posedge clk or negedge reset) begin
        if(!reset)
            post_for_counter <= 0;
        else if(state == POST_FORWARD) begin
            if(post_for_counter < PAUSE_CYCLES)
                post_for_counter <= post_for_counter + 1;
        end
            else
                post_for_counter <= 0;
    end
	 
	always @(*) begin
		move_done = (state == WAIT);
		end
    always @(*) begin
        
    end


    always @(posedge clk) begin
       if(stopped) uturn_counter <= uturn_counter + 1;
       else uturn_counter <= 0; 
    end
    
    
     //arithmetic shift right 

    // =====================================================
    // FSM combinational logic
    // =====================================================
    always @(*) begin
        enable           = 1'b0;
        to_motordriver   = steer_cmd;
        next_state       = state;
        mpi_start        = 1'b0;

        case (state)

            IDLE: begin
                stopped = 1'b0;
                if (current_move != STOP)
                    next_state = MOVING;
                uturn_done = 1'b0;
            end

            MOVING: begin
                enable = 1'b1;
                stopped = 1'b0;
                if (maze_done) to_motordriver = STOP;
                else begin 
                    case (current_move)
                        FORWARD: begin
                            if  (avg_turn >= FWD_TICKS || dist2 < 125)
                                next_state = WAIT;
                        end

                        LEFT: begin
                            if (avg_turn >= LTICK_90)
                                next_state = POST_FORWARD;

                        end

                        RIGHT: begin
                            if (avg_turn >= RTICK_90)
                                next_state = POST_FORWARD;
                            end


                        UTURN : begin
                              mpi_start = 1'b1;
                              to_motordriver = STOP;
                              stopped = 1'b1;
                              if(uturn_counter >= 4) mpi_start = 1'b0;
                              //if(mpi_done) 
                              //begin
                              if(uturn_counter >= 350_000_000) begin
                                to_motordriver = UTURN;
                                if(avg_turn >= TICK_180) next_state = POST_FORWARD_UTURN;
                              end

                              //end
                              //uturn_done = 1'b0;
                        end

                        UTURN_NMPI : begin
                            to_motordriver = UTURN;
                            if (avg_turn >= TICK_180)
                                next_state = POST_FORWARD_UTURN;
                        end

                        default:
                            next_state = IDLE;
                    endcase
                end
            end

            POST_FORWARD: begin
            /*  if (post_for_counter < PAUSE_CYCLES)
                enable = 1'b0;   // pause
              else  */
                enable         = 1'b1;
                stopped = 1'b0;
                to_motordriver = FORWARD;

              if (avg_turn >= POST_FWD_TICKS || dist2 < 120)
                next_state = DONE;
                
            end

            POST_FORWARD_UTURN: begin
                enable         = 1'b1;
                to_motordriver = FORWARD;
                stopped = 1'b0;

              if (avg_turn >= POST_FWD_TICKS || dist2 < 120)
                next_state = DONE;
                
            end

				DONE: begin
					enable = 1'b0;
                    stopped = 1'b0;
					next_state = WAIT;
				end

            WAIT: begin
                stopped = 1'b0;
              if(maze_ack)
                next_state = IDLE;
            end

        endcase
    end
/*
    reg [15:0] wall_d1,wall_d2;
    reg [31:0] align_start_ticks
    reg align_phase;

    wire [15:0] align_dist = (current_move == LEFT) ? dist3 : (currrent_move == RIGHT) ? dist1 : dist3;
*/

/// ===================
// Wall filtering (low-pass)
// ===================
reg [15:0] distL_f, distR_f;

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        distL_f <= 16'd0;
        distR_f <= 16'd0;
    end else begin
        distL_f <= (distL_f + dist3) >> 1;  // left sensor
        distR_f <= (distR_f + dist1) >> 1;  // right sensor
    end
end


// ===================
// Wall detection
// ===================
localparam [15:0] WALL_VALID = 16'd150;  // 30 cm threshold

wire left_wall  = (distL_f < WALL_VALID);
wire right_wall = (distR_f < WALL_VALID);


// ===================
// Tracking Parameters
// ===================
localparam signed [17:0] SWALL_TARGET = 18'sd200;  // desired 8cm offset
localparam signed [17:0] SMALL_ERR    = 18'sd20;
localparam signed [17:0] MED_ERR      = 18'sd40;


// ===================
// Unified Wall Error
// ===================
reg signed [17:0] wall_err;

always @(*) begin
    wall_err = 18'sd0;

    if (left_wall && right_wall) begin
        // corridor centering
        wall_err = $signed({distR_f}) - $signed({distL_f});
    end
    else if (left_wall) begin
        // track left wall
        wall_err = $signed({1'b0, dist3}) - SWALL_TARGET;
    end
    else if (right_wall) begin
        // track right wall
        wall_err = SWALL_TARGET - $signed({1'b0, dist1});
    end
end


reg [24:0] sample_timer;
reg [15:0] distL_slow, distR_slow;


always @(posedge clk or negedge reset) begin
    if (!reset) begin
        sample_timer <= 0;
        distL_slow   <= 0;
        distR_slow   <= 0;
        deltaL_reg   <= 0;
        deltaR_reg   <= 0;
    end else begin
        if (sample_timer < 500_000) begin
            sample_timer <= sample_timer + 1;
        end else begin
            sample_timer <= 0;
            
            // 1. Calculate Delta using the Snapshot from 50ms ago
            deltaL_reg <= $signed({dist3}) - $signed({distL_slow});
            deltaR_reg <= $signed({dist1}) - $signed({distR_slow});
            
            // 2. Update the Snapshot for the NEXT 50ms
            distL_slow <= dist3;
            distR_slow <= dist1;
        end
    end
end

// Connect the wires to these registered values
assign deltaL = deltaL_reg;
assign deltaR = deltaR_reg;
/*
// ===================
// Steering Logic
// ===================
always @(*) begin
    // default
    steer_cmd = current_move;

    // force straight in POST_FORWARD
    if (state == POST_FORWARD)
        steer_cmd = FORWARD;

    // apply correction only during straight movement
    if ((state == MOVING && current_move == FORWARD) ||
        (state == POST_FORWARD)) begin

        if (left_wall && right_wall) begin

           // if((deltaR <= -4 || deltaR >= 4) || (deltaL <= -4 || deltaL >= 4)) begin
        if(state == POST_FORWARD && avg_turn >2000) begin
            if (wall_err > MED_ERR)
                steer_cmd = DRIFT_LEFT;

            else if (wall_err < -MED_ERR)
                steer_cmd = DRIFT_RIGHT;

            else if (wall_err > SMALL_ERR)
                steer_cmd = DRIFT_LEFT;

            else if (wall_err < -SMALL_ERR)
                steer_cmd = DRIFT_RIGHT;

            else
                steer_cmd = FORWARD;
        // end
        end

        else if(state == FORWARD && avg_turn <1000) begin
            if (wall_err > MED_ERR)
                steer_cmd = DRIFT_LEFT;

            else if (wall_err < -MED_ERR)
                steer_cmd = DRIFT_RIGHT;

            else if (wall_err > SMALL_ERR)
                steer_cmd = DRIFT_LEFT;

            else if (wall_err < -SMALL_ERR)
                steer_cmd = DRIFT_RIGHT;

            else
                steer_cmd = FORWARD;
        // end
        end
        end

        /*else if(left_wall && !right_wall) begin

            if(dist3 < 73)
                steer_cmd = DRIFT_LEFT;
            else if(dist3 > 73)
                steer_cmd = DRIFT_RIGHT;
        end

        
        else if(!left_wall && right_wall) begin

            if(dist1 < 73)
                steer_cmd = DRIFT_RIGHT;
            else if(dist1 > 73)
                steer_cmd = DRIFT_LEFT;


        end

    end
end

endmodule */

// ===================
// Correction Window Control
// ===================

localparam [31:0] CORR_WINDOW_FWD = 32'd1000;
localparam [31:0] CORR_WINDOW_PFWD = 32'd2000;

reg correction_active;

always @(*) begin
    correction_active = 1'b0;

    case (state)

        // FORWARD → first 2000 ticks
        MOVING: begin
            if (current_move == FORWARD) begin
                if (avg_turn < CORR_WINDOW_FWD)
                    correction_active = 1'b1;
            end
        end

        // POST_FORWARD → last 2000 ticks
        POST_FORWARD: begin
            if (avg_turn > (POST_FWD_TICKS - CORR_WINDOW_PFWD))
                correction_active = 1'b1;
        end

        default: correction_active = 1'b0;

    endcase
end

// ===================
// Steering Logic (Windowed Correction)
// ===================
always @(*) begin
    // default command
    steer_cmd = current_move;

    // force straight base command in POST_FORWARD
    if (state == POST_FORWARD)
        steer_cmd = FORWARD;

    // Apply correction only inside correction window
    if (correction_active) begin

        // ---------------- Corridor Centering ----------------
        if (left_wall && right_wall) begin

    // Only correct if not parallel
    if ((deltaL > 2 || deltaL < -2) ||
        (deltaR > 2 || deltaR < -2)) begin

        if (wall_err > MED_ERR)
            steer_cmd = DRIFT_LEFT;

        else if (wall_err < -MED_ERR)
            steer_cmd = DRIFT_RIGHT;

        else if (wall_err > SMALL_ERR)
            steer_cmd = DRIFT_LEFT;

        else if (wall_err < -SMALL_ERR)
            steer_cmd = DRIFT_RIGHT;

        else
            steer_cmd = FORWARD;
    end
    else begin
        // parallel → no correction
        steer_cmd = FORWARD;
    end
end

        // ---------------- Left Wall Tracking ----------------
        else if (left_wall && !right_wall) begin

            if (deltaL > 2 || deltaL < -2) begin
                if (dist3 < 73)
                    steer_cmd = DRIFT_LEFT;
                else if (dist3 > 73)
                    steer_cmd = DRIFT_RIGHT;
                else
                    steer_cmd = FORWARD;
            end
            else
                steer_cmd = FORWARD;
        end

        // ---------------- Right Wall Tracking ----------------
        else if (!left_wall && right_wall) begin

            if (deltaR > 2 || deltaR < -2) begin
                if (dist1 < 73)
                    steer_cmd = DRIFT_RIGHT;
                else if (dist1 > 73)
                    steer_cmd = DRIFT_LEFT;
                else
                    steer_cmd = FORWARD;
            end
            else
                steer_cmd = FORWARD;
        end
    end
end

endmodule
