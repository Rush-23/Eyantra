/*
# Team ID:          1024
# Theme:            MazeSolver Bot
# Author List:      Rushil V, Indiran T, Sathiya Naarayanan C, Charan Karthick A S
# Filename:         servo
# File Description: code for servo motor control
# Global variables: N/A
*/

module servo (
    input  wire clk_50M,
    input  wire reset,

    input  wire servo_start,   // from mpi_controller
    input  wire servo_release, // from mpi_controller
    output reg  servo_pwm,
    output reg  servo_done,
    output wire dipped
);

    // ================= PWM parameters =================
    localparam PERIOD_CNT = 1_000_000; // 20ms @ 50MHz
    localparam PULSE_UP   = 50_000;    // 1ms  (HOME)
    localparam PULSE_DOWN = 95_000;   // 2ms  (DIP)

    // ====== SPEED CONTROL (KEY ADDITION) ======
    localparam PULSE_STEP = 20'd5;   // smaller = slower (≈10 µs per step)

    // ================= Time durations (ms) =================
    localparam DIP_TIME     = 300;
    localparam HOLD_TIME    = 6000;
    localparam RETRACT_TIME = 300;

    // ================= FSM states =================
    localparam IDLE    = 0,
               DIP     = 1,
               HOLD    = 2,
               RETRACT = 3,
               DONE    = 4;

    reg [2:0]  state;
    reg [19:0] pwm_cnt;
    reg [19:0] pulse_width;

    // ================= 1 ms tick generator =================
    reg [15:0] us_cnt;
    reg        ms_tick;
    reg [15:0] ms_cnt;
    reg [15:0] phase_time;

    // ================= PWM generation =================
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            pwm_cnt   <= 0;
            servo_pwm <= 0;
        end else begin
            pwm_cnt <= (pwm_cnt == PERIOD_CNT-1) ? 0 : pwm_cnt + 1;
            servo_pwm <= (pwm_cnt < pulse_width);
        end
    end

    // ================= 1 ms tick =================
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            us_cnt  <= 0;
            ms_tick <= 0;
        end else begin
            if (us_cnt == 50_000-1) begin
                us_cnt  <= 0;
                ms_tick <= 1;
            end else begin
                us_cnt  <= us_cnt + 1;
                ms_tick <= 0;
            end
        end
    end

    //wire dipped;
    assign dipped = (state == HOLD);
    

    // ================= Servo FSM =================
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            state       <= IDLE;
            pulse_width <= PULSE_UP;
            servo_done  <= 0;
            ms_cnt      <= 0;
        end else begin
            //servo_done <= 0;
            if (ms_tick) ms_cnt <= ms_cnt + 1;

            case (state)

                IDLE: begin
                    pulse_width <= PULSE_DOWN;
                    ms_cnt <= 0;
                    if (servo_start) begin
                        state <= DIP;
                        phase_time <= DIP_TIME;
                    end
                end

                // -------- SLOW DIP (RAMP DOWN) --------
                DIP: begin
                    if (pulse_width < PULSE_UP)
                        pulse_width <= pulse_width + PULSE_STEP;
                    else
                        pulse_width <= PULSE_UP;

                    if (ms_cnt >= phase_time) begin
                        state <= HOLD;
                        phase_time <= HOLD_TIME;
                        ms_cnt <= 0;
                    end
                end

                // -------- HOLD POSITION --------
                HOLD: begin
                    pulse_width <= PULSE_UP;
                    if (servo_release) begin
                        state <= RETRACT;
                        phase_time <= RETRACT_TIME;
                        ms_cnt <= 0;
                    end
                end

                // -------- SLOW RETRACT (RAMP UP) --------
                RETRACT: begin
                    if (pulse_width > PULSE_DOWN)
                        pulse_width <= pulse_width - PULSE_STEP;
                    else
                        pulse_width <= PULSE_UP;

                    if (ms_cnt >= phase_time)
                        state <= DONE;
                end

                DONE: begin
                    servo_done <= 1'b1;   // 1-cycle pulse
                    state <= IDLE;
                end

            endcase
        end
    end
endmodule