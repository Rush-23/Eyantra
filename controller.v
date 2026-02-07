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
    localparam FWD_TICKS = 32'd4800, LTICK_90 = 32'd1425, RTICK_90 =32'd1400, TICK_180 = 32'd2850, POST_FWD_TICKS = 32'd5420;

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
wire signed [15:0] wall_error;
assign wall_error = $signed(distR_f) - $signed(distL_f);

// 2cm deadband
localparam signed [15:0] WALL_TOL = 16'sd5000;

reg [15:0] corr_timer;

always @(posedge clk or negedge reset) begin
    if (!reset)
        corr_timer <= 0;
    else if (state == MOVING || state == POST_FORWARD)
        corr_timer <= corr_timer + 1;
    else if (steer_cmd != FORWARD)
        corr_timer <= 0;
    else 
        corr_timer <= 0;
end





wire allow_correction = (corr_timer > 16'd50000); // ~1ms at 50MHz



//single wall parameters
localparam SWALL_TARGET = 16'd115;   // 12cm
localparam SWALL_TOL    = 16'd15;    // 1.5cm deadband
localparam SWALL_MAX    = 16'd250;   // wall valid below 25cm

wire signed [16:0] err_left  = $signed(dist3) - SWALL_TARGET;
wire signed [16:0] err_right = SWALL_TARGET - $signed(dist1);


always @(*) begin
    steer_cmd = current_move;   // default: do what FSM wants

    if ((state == MOVING || state == POST_FORWARD) &&
        (current_move == FORWARD) && allow_correction) begin
    
    if(left_wall && right_wall) begin
        if (wall_error > WALL_TOL)
            steer_cmd = DRIFT_LEFT;   // too close to left wall
        else if (wall_error < -WALL_TOL)
            steer_cmd = DRIFT_RIGHT;    // too close to right wall
        else
            steer_cmd = FORWARD;
    end 

    /*else if (left_wall && !right_wall) begin
    // Track left wall
    if (err_left > WALL_TOL)
        steer_cmd = LEFT;      // too far → move closer
    else if (err_left < -WALL_TOL)
        steer_cmd = RIGHT;     // too close → move away
    end

    else if (right_wall && !left_wall) begin
    // Track right wall
    if (err_right > WALL_TOL)
        steer_cmd = RIGHT;     // too far → move closer
    else if (err_right < -WALL_TOL)
        steer_cmd = LEFT;      // too close → move away
    end */

    end

end 

endmodule



