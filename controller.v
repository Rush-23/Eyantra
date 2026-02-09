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
    output reg  uturn_done
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
    localparam FWD_TICKS = 32'd4950, LTICK_90 = 32'd1425, RTICK_90 =32'd1450, TICK_180 = 32'd2850, POST_FWD_TICKS = 32'd5490;

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

// ===================
// Wall filtering
// ===================
reg [15:0] distL_f, distR_f;

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        distL_f <= 0;
        distR_f <= 0;
    end else begin
        distL_f <= (distL_f + dist3) >> 1;   // left ultrasonic
        distR_f <= (distR_f + dist1) >> 1;   // right ultrasonic
    end
end

// ===================
// Wall detection
// ===================
wire left_wall  = (distL_f < 300);   // 25cm
wire right_wall = (distR_f < 300);

// ===================
// Error computation
// ===================
//wire signed [15:0] wall_error;
//assign wall_error = $signed(distR_f) - $signed(distL_f);

// 2cm deadband
//localparam signed [15:0] WALL_TOL = 16'sd15;

reg [15:0] corr_timer;

always @(posedge clk or negedge reset) begin
    if (!reset)
        corr_timer <= 0;
    else if ((state == MOVING && current_move == FORWARD)|| state == POST_FORWARD) //cchanged
        corr_timer <= corr_timer + 1;
    else if (steer_cmd != FORWARD)
        corr_timer <= 0;
    else 
        corr_timer <= 0;
end

wire allow_correction = (corr_timer > 16'd5_000_000); // ~1ms at 50MHz

//single wall parameters
// ==========================
// Single/Dual wall pulse control
// ==========================

// tune
localparam [15:0] SWALL_TARGET = 16'd115;   // 12cm
localparam [15:0] SWALL_TOL    = 16'd20;    // 2cm
localparam [15:0] SWALL_MAX    = 16'd250;   // 25cm

localparam signed [15:0] WALL_TOL = 16'sd20; // dual wall deadband (~2cm)

// timing @ 50MHz
localparam [19:0] SAMPLE_TICKS = 20'd500000;  // 10ms
localparam [19:0] PULSE_TICKS  = 20'd150000;  // 3ms
localparam signed [16:0] SLOPE_TOL = 17'sd10;
reg [19:0] sample_timer;
reg [15:0] d1_left, d2_left;
reg [15:0] d1_right, d2_right;

reg [2:0] corr_cmd;

reg [19:0] pulse_timer;

reg [15:0] prev_distL;
reg [15:0] prev_distR;

reg        pulse_active;
reg [2:0]  pulse_cmd;


// errors
wire signed [15:0] wall_error = $signed(distR_f) - $signed(distL_f);

wire signed [16:0] e_left  = $signed(distL_f) - $signed(SWALL_TARGET);
wire signed [16:0] e_right = $signed(SWALL_TARGET) - $signed(distR_f);

wire signed [16:0] d_left  = $signed(distL_f) - $signed(prev_distL);
wire signed [16:0] d_right = $signed(distR_f) - $signed(prev_distR);

wire left_valid  = left_wall  && (distL_f < SWALL_MAX);
wire right_valid = right_wall && (distR_f < SWALL_MAX);


always @(posedge clk or negedge reset) begin
    if (!reset) begin
        sample_timer <= 0;
        d1_left  <= 0;
        d1_right <= 0;
        d2_left  <= 0;
        d2_right <= 0;
        corr_cmd <= FORWARD;

    end else begin
        corr_cmd <= FORWARD;  // default

        if ((state == MOVING || state == POST_FORWARD) &&
            (current_move == FORWARD)) begin

            // timer
            if (sample_timer < SAMPLE_TICKS)
                sample_timer <= sample_timer + 1;
            else
                sample_timer <= 0;

            // take t1
            if (sample_timer == 0) begin
                d1_left  <= distL_f;
                d1_right <= distR_f;
            end

            // take t2 + decide
            if (sample_timer == SAMPLE_TICKS) begin
                d2_left  <= distL_f;
                d2_right <= distR_f;

                // LEFT wall only: use slope
                if (left_wall && !right_wall) begin
                    if (($signed(distL_f) - $signed(d1_left)) < -SLOPE_TOL)
                        corr_cmd <= DRIFT_RIGHT; // getting closer -> move away
                    else if (($signed(distL_f) - $signed(d1_left)) > SLOPE_TOL)
                        corr_cmd <= DRIFT_LEFT;  // getting farther -> move toward
                end

                // RIGHT wall only: use slope
                else if (right_wall && !left_wall) begin
                    if (($signed(distR_f) - $signed(d1_right)) < -SLOPE_TOL)
                        corr_cmd <= DRIFT_LEFT;  // getting closer -> move away
                    else if (($signed(distR_f) - $signed(d1_right)) > SLOPE_TOL)
                        corr_cmd <= DRIFT_RIGHT; // getting farther -> move toward
                end
            end

        end else begin
            sample_timer <= 0;
        end
    end
end

always@(*) begin 
    steer_cmd = (corr_cmd != FORWARD) ? corr_cmd : current_move;
end





// ==========================
// ONE sequential block owns pulse_cmd + pulse_active
// ==========================
/*always @(posedge clk or negedge reset) begin
    if (!reset) begin
        sample_timer <= 0;
        pulse_timer  <= 0;

        prev_distL   <= 0;
        prev_distR   <= 0;

        pulse_active <= 0;
        pulse_cmd    <= FORWARD;

    end else begin

        // Default: no pulse unless triggered
        // (do NOT set pulse_active=0 here every cycle, it will kill pulses)

        if ((state == MOVING || state == POST_FORWARD) &&
            (current_move == FORWARD)) begin

            // -------------------------
            // sample timer
            // -------------------------
            if (sample_timer < SAMPLE_TICKS)
                sample_timer <= sample_timer + 1;
            else
                sample_timer <= 0;

            // -------------------------
            // pulse timer
            // -------------------------
            if (pulse_active) begin
                if (pulse_timer < PULSE_TICKS)
                    pulse_timer <= pulse_timer + 1;
                else begin
                    pulse_timer  <= 0;
                    pulse_active <= 0;
                    pulse_cmd    <= FORWARD;
                end
            end

            // -------------------------
            // At sample boundary:
            // - store prev distances
            // - if not pulsing, decide whether to start a pulse
            // -------------------------
            if (sample_timer == SAMPLE_TICKS) begin
                prev_distL <= distL_f;
                prev_distR <= distR_f;

                // Only trigger a new pulse if none active
                if (!pulse_active) begin

                    // default: no correction
                    pulse_cmd <= FORWARD;

                    // ==================================
                    // 1) DUAL WALL (highest priority)
                    // ==================================
                    if (left_valid && right_valid) begin
                        if (wall_error > WALL_TOL) begin
                            pulse_cmd    <= DRIFT_LEFT;
                            pulse_active <= 1;
                            pulse_timer  <= 0;
                        end else if (wall_error < -WALL_TOL) begin
                            pulse_cmd    <= DRIFT_RIGHT;
                            pulse_active <= 1;
                            pulse_timer  <= 0;
                        end
                    end

                    // ==================================
                    // 2) SINGLE WALL: LEFT
                    // ==================================
                    else if (left_valid && !right_valid) begin

                        if (e_left < -$signed(SWALL_TOL)) begin
                            pulse_cmd    <= DRIFT_RIGHT;
                            pulse_active <= 1;
                            pulse_timer  <= 0;
                        end else if (e_left > $signed(SWALL_TOL)) begin
                            pulse_cmd    <= DRIFT_LEFT;
                            pulse_active <= 1;
                            pulse_timer  <= 0;
                        end

                        // trend damping (optional)
                        else if (d_left < -17'sd8) begin
                            pulse_cmd    <= DRIFT_RIGHT;
                            pulse_active <= 1;
                            pulse_timer  <= 0;
                        end
                    end

                    // ==================================
                    // 3) SINGLE WALL: RIGHT
                    // ==================================
                    else if (right_valid && !left_valid) begin

                        if (e_right < -$signed(SWALL_TOL)) begin
                            pulse_cmd    <= DRIFT_LEFT;
                            pulse_active <= 1;
                            pulse_timer  <= 0;
                        end else if (e_right > $signed(SWALL_TOL)) begin
                            pulse_cmd    <= DRIFT_RIGHT;
                            pulse_active <= 1;
                            pulse_timer  <= 0;
                        end

                        // trend damping (optional)
                        else if (d_right < -17'sd8) begin
                            pulse_cmd    <= DRIFT_LEFT;
                            pulse_active <= 1;
                            pulse_timer  <= 0;
                        end
                    end

                end
            end

        end else begin
            // Not in forward move: reset correction state
            sample_timer <= 0;
            pulse_timer  <= 0;

            pulse_active <= 0;
            pulse_cmd    <= FORWARD;
        end
    end
end

always@(*) begin
    if(pulse_active)
        steer_cmd = pulse_cmd;
    else
        steer_cmd = current_move;
end /*



   /* else if (left_wall && !right_wall) begin
    // Track left wall
    if (err_left > SWALL_TOL)
        steer_cmd = DRIFT_RIGHT;      // too far → move closer
    else if (err_left < -SWALL_TOL)
        steer_cmd = DRIFT_LEFT;     // too close → move away
    end

    else if (right_wall && !left_wall) begin
    // Track right wall
    if (err_right > SWALL_TOL)
        steer_cmd = DRIFT_LEFT;     // too far → move closer
    else if (err_right < -SWALL_TOL)
        steer_cmd = DRIFT_RIGHT;      // too close → move away
    end */

    //end
	// end

   /* // ================================
    // Single Wall Correction Parameters
    // ================================
    localparam integer CORR_TICKS = 5_000_000; // ~30 ms
    localparam signed [16:0] DELTA_TOL = 17'sd15;     // ~1 cm noise band
    localparam [15:0] WALL_VALID = 16'd250;          // 25 cm max wall distance

    reg [25:0] corr_cnt;
    reg        corr_tick;
    reg [15:0] prev_wall_dist;
    reg signed [16:0] delta_wall;

    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            corr_cnt  <= 0;
            corr_tick <= 1'b0;
        end else if (corr_cnt >= CORR_TICKS) begin
            corr_cnt  <= 0;
            corr_tick <= 1'b1;
        end else begin
            corr_cnt  <= corr_cnt + 1'b1;
            corr_tick <= 1'b0;
        end
    end
    wire left_wall_only  = (dist3 < WALL_VALID) && (dist1 >= WALL_VALID);
    wire right_wall_only = (dist1 < WALL_VALID) && (dist3 >= WALL_VALID);
    
    wire [15:0] wall_dist =
        left_wall_only  ? dist3 :
        right_wall_only ? dist1 :
        16'd0;

    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            prev_wall_dist <= 16'd0;
            delta_wall     <= 17'sd0;
        end else if (corr_tick && (left_wall_only || right_wall_only)) begin
            delta_wall     <= $signed(wall_dist) - $signed(prev_wall_dist);
            prev_wall_dist <= wall_dist;
        end
    end
    always @(*) begin
        steer_cmd = current_move; // default: FSM decides
    
        if ((state == MOVING && current_move == FORWARD) ||
            (state == POST_FORWARD) &&
            (left_wall_only || right_wall_only)) begin
            
            if (delta_wall > DELTA_TOL) begin
                // drifting away from wall
                steer_cmd = left_wall_only ? DRIFT_LEFT : DRIFT_RIGHT;
            end
            else if (delta_wall < -DELTA_TOL) begin
                // drifting too close
                steer_cmd = left_wall_only ? DRIFT_RIGHT : DRIFT_LEFT;
            end
            else begin
                steer_cmd = FORWARD;
            end
        end
    end*/


endmodule



