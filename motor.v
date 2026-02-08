/*
# Team ID:          1024
# Theme:            MazeSolver Bot
# Author List:      Rushil V, Indiran T, Sathiya Naarayanan C, Charan Karthick A S
# Filename:         motor
# File Description: Command centre to give signals to the motor driver
# Global variables: N/A
*/

module motor (
    input  wire        clk_50,
    input  wire        rst,

    input  wire [3:0]  move_cmd,
    input  wire        enable,

    output wire        l_in1,
    output wire        l_in2,
    output wire        r_in1,
    output wire        r_in2,
    output reg         en_l,
    output reg         en_r
);

    localparam STOP        = 4'b0000,
               FORWARD     = 4'b0001,
               LEFT        = 4'b0010,
               RIGHT       = 4'b0011,
               UTURN       = 4'b0100,
               DRIFT_LEFT   = 4'b0101,
               DRIFT_LEFT_HARD   = 4'b0110,
               DRIFT_RIGHT = 4'b0111,
               DRIFT_RIGHT_HARD  = 4'b1000,  // example, adjust encodings
               REVERSE     = 4'b1001;

    reg l1, l2, r1, r2;

    assign l_in1 = l1;
    assign l_in2 = l2;
    assign r_in1 = r1;
    assign r_in2 = r2;

    // =========================
    // PWM generator (8-bit)
    // =========================
    reg [7:0] count;

    always @(posedge clk_50 or negedge rst) begin
        if (!rst)
            count <= 8'd0;
        else
            count <= count + 1'b1;
    end

    wire speed_10 = (count < 8'd128);
    wire speed_25 = (count < 8'd180);
    wire speed_50 = (count < 8'd180);
    wire speed_75 = (count < 8'd165);
    wire speed_40 = (count < 8'd150); 
    wire speed_60 = (count < 8'd170);


    // =========================
    // Direction control
    // =========================
   always @(posedge clk_50 or negedge rst) begin
    if (!rst) begin
        l1 <= 1'b0;
        l2 <= 1'b0;
        r1 <= 1'b0;
        r2 <= 1'b0;
    end else if (enable) begin
        case (move_cmd)
            FORWARD: begin
                l1 <= 1; l2 <= 0;
                r1 <= 1; r2 <= 0;
            end

            REVERSE: begin
                l1 <= 0; l2 <= 1;
                r1 <= 0; r2 <= 1;
            end

            LEFT: begin
                l1 <= 1; l2 <= 0;
                r1 <= 0; r2 <= 1;
            end

            RIGHT: begin
                l1 <= 0; l2 <= 1;
                r1 <= 1; r2 <= 0;
            end

            DRIFT_LEFT,
            DRIFT_LEFT_HARD: begin
                l1 <= 1; l2 <= 0;
                r1 <= 0; r2 <= 1;
            end

            DRIFT_RIGHT,
            DRIFT_RIGHT_HARD: begin
                l1 <= 0; l2 <= 1;
                r1 <= 1; r2 <= 0;
            end

            UTURN: begin
                l1 <= 1; l2 <= 0;
                r1 <= 0; r2 <= 1;
            end

            STOP: begin
                l1 <= 0; l2 <= 0;
                r1 <= 0; r2 <= 0;
            end

            default: begin
                l1 <= 0; l2 <= 0;
                r1 <= 0; r2 <= 0;
            end
        endcase
    end else begin
        l1 <= 0; l2 <= 0;
        r1 <= 0; r2 <= 0;
    end
end


    // =========================
    // Speed control (PWM enable)
    // =========================
    always @(*) begin
        en_l = 1'b0;
        en_r = 1'b0;

        if (enable) begin
            case (move_cmd)
                FORWARD, REVERSE: begin
                    en_l = speed_50;
                    en_r = speed_50;
                end

                LEFT: begin
                    en_l = speed_25;
                    en_r = speed_25;
                end

                RIGHT: begin
                    en_l = speed_25;
                    en_r = speed_25;
                end

                DRIFT_LEFT: begin
                    en_l = speed_40;
                    en_r = speed_50;
                end

                DRIFT_LEFT_HARD: begin
                    en_l = speed_60;
                    en_r = speed_50;
                end

                DRIFT_RIGHT: begin
                    en_l = speed_50;
                    en_r = speed_40;
                end

                DRIFT_RIGHT_HARD: begin
                    en_l = speed_50;
                    en_r = speed_60;
                end

                UTURN: begin
                    en_l = speed_25;
                    en_r = speed_25;
                end

                default: begin
                    en_l = 1'b0;
                    en_r = 1'b0;
                end
            endcase
        end
    end

endmodule
