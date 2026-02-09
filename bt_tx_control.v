/*
# Team ID:          1024
# Theme:            MazeSolver Bot
# Author List:      Rushil V, Indiran T, Sathiya Naarayanan C, Charan Karthick A S
# Filename:         bt_tx_control
# File Description: Bluetooth transmission command centre
# Global variables: N/A
*/

module bt_tx_control (
    input  wire clk,
    input  wire rst,     // active LOW reset
    //input  wire [15:0] dist1,
    input  wire uart_busy,
    input [3:0] dead_count,
    input moist_done,
    input [7:0] temp_int,temp_decimal,hum_int,hum_decimal,

    output reg  tx_start,
    output reg  [7:0] tx_data,
    input  wire [1:0] msg_type,
    output reg  btx_done,
    input  wire send_start,
    input  wire mord,
    input  wire maze_done
);

    reg [7:0] state;
    reg [15:0] delay_cnt;
    

    localparam MSG_MPIM = 2'd0,
               MSG_MM   = 2'd1,
               MSG_TH   = 2'd2,
               MSG_END  = 2'd3;

    localparam IDLE = 0, S_NL = 1, S_NL1 = 60, S_NL2 = 61;  

    reg [3:0] prev_dead_count = 0;

    always@(posedge clk)
        prev_dead_count <= dead_count;



    /*localparam S_M = 0, S_I = 1, S_P = 2, S_M2 = 3, S_EQ = 4,
               S_NL1 = 5, HUM = 6, IDLE = 7, S_H0 = 8, S_NL = 9, MPI = 10;*/

    function [7:0] to_hex;
        input [3:0] v;
        begin
            if (v < 10) to_hex = 8'h30 + v;
            else        to_hex = 8'h41 + (v - 10);
        end
    endfunction

    reg transmit;
    //always @(posedge clk or negedge rst) begin
    localparam  DELAY = 16'd100000; // Adjust this value to control the delay between transmissions


    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state <= IDLE;
            tx_start <= 0;
            tx_data <= 0;
            delay_cnt <= 0;
            btx_done <= 0;
        end else begin
            tx_start <= 0;
            
           if (!uart_busy) begin

                case (state)
                    IDLE: begin
                    //btx_done <=0;
                    //tx_start <= 0;
                    if(maze_done)
                        state <= 114;
                    if (send_start) begin
                        case (msg_type)
                            MSG_MPIM: state <= 2;
                          //  MSG_MM:   state <= 12;
                          /*  MSG_TH:   state <= 40; */
                            
                        endcase
                    end
                end

                    // MPIM Message
                    2:  begin 
                        //if(dead_count != prev_dead_count) 
                        /*begin*/ tx_data <= "M"; tx_start <= 1; state <= 80; /*end */
                        //else state <= 2; // If no change in death count, just send a newline 
                        end
                    80: begin tx_data <= "X"; tx_start <= 1; state <= 3; end   // 1-cycle 
                    3:  begin tx_data <= "P"; tx_start <= 1;  state <= 81; end
                    81: begin tx_data <= "X"; tx_start <= 1; state <= 4; end
                    4:  begin tx_data <= "I"; tx_start <= 1; btx_done <= 1'b1; state <= 82; end
                    82: begin tx_data <= "X"; tx_start <= 1; state <= 5; end
                    5:  begin tx_data <= "M"; tx_start <= 1; state <= 83; end
                    83: begin tx_data <= "X"; tx_start <= 1; state <= 6; end
                    6:  begin tx_data <= "-"; tx_start <= 1; state <= 84; end
                    84: begin tx_data <= "X"; tx_start <= 1; state <= 7; end
                    7:  begin tx_data <= to_hex(dead_count); tx_start <= 1; state <= 85; end
                    85: begin tx_data <= "X"; tx_start <= 1; state <= 8; end
                    8:  begin tx_data <= "-"; tx_start <= 1; state <= 86; end
                    86: begin tx_data <= "X"; tx_start <= 1; state <= 9; end
                    9:  begin tx_data <= "#"; tx_start <= 1; state <= 87; end
                    87: begin tx_data <= "X"; tx_start <= 1; state <= S_NL; end
                    S_NL: begin tx_data <= 8'h0A; tx_start <= 1; state <= 18;end
                    
                    //Moisture message
                    18: begin tx_start <= 0; if(moist_done) state <= 12; end
                    12: begin tx_data <= "M"; tx_start <= 1; state <= 88; end
                    88: begin tx_data <= "X"; tx_start <= 1; state <= 13; end
                    13: begin tx_data <= "M"; tx_start <= 1; state <= 89; end
                    89: begin tx_data <= "X"; tx_start <= 1; state <= 14; end
                    14: begin tx_data <= "-"; tx_start <= 1; state <= 90; end
                    90: begin tx_data <= "X"; tx_start <= 1; state <= 15; end
                    15: begin tx_data <= to_hex(dead_count); tx_start <= 1; state <= 91; end
                    91: begin tx_data <= "X"; tx_start <= 1; state <= 16; end
                    16: begin tx_data <= "-"; tx_start <= 1; state <= 92; end
                    92: begin tx_data <= "X"; tx_start <= 1; state <= 17; end
                    17: begin tx_data <= (mord ? "M" : "D");tx_start <= 1;state <= 93;end
                    93: begin tx_data <= "X"; tx_start <= 1; state <= 56; end
                    56: begin tx_data <= "-"; tx_start <= 1; state <= 111; end
                    111: begin tx_data <= "X"; tx_start <= 1; state <= 19; end
                    19: begin tx_data <= "#"; tx_start <= 1; state <= 94; end
                    94: begin tx_data <= "X"; tx_start <= 1; state <= S_NL1; end
                    S_NL1: begin tx_data <= 8'h0A; tx_start <= 1; state <= 112; end

                    //Temperature and Humidity message
                    112: begin tx_data <= "X"; tx_start <= 1; state <= 40; end
                    40: begin tx_data <= "T"; tx_start <= 1; state <= 95; end
                    95: begin tx_data <= "X"; tx_start <= 1; state <= 41; end
                    41: begin tx_data <= "H"; tx_start <= 1; state <= 96; end
                    96: begin tx_data <= "X"; tx_start <= 1; state <= 42; end
                    42: begin tx_data <= "-"; tx_start <= 1; state <= 97; end
                    97: begin tx_data <= "X"; tx_start <= 1; state <= 43; end
                    43: begin tx_data <= to_hex(dead_count); tx_start <= 1; state <= 98; end
                    98: begin tx_data <= "X"; tx_start <= 1; state <= 44; end
                    44: begin tx_data <= "-"; tx_start <= 1; state <= 99; end
                    99: begin tx_data <= "X"; tx_start <= 1; state <= 45; end
                    45: begin tx_data <= to_hex(temp_int/10); tx_start <= 1; state <= 100; end
                    100: begin tx_data <= "X"; tx_start <= 1; state <= 46; end
                    46: begin tx_data <= to_hex(temp_int%10); tx_start <= 1; state <= 101; end                    
                    101: begin tx_data <= "X"; tx_start <= 1; state <= 47; end
                    47: begin tx_data <= "."; tx_start <= 1; state <= 102; end
                    102: begin tx_data <= "X"; tx_start <= 1; state <= 48; end
                    48: begin tx_data <= to_hex(temp_decimal); tx_start <= 1; state <= 103; end
                    103: begin tx_data <= "X"; tx_start <= 1; state <= 49; end
                    49: begin tx_data <= "-"; tx_start <= 1; state <= 104; end 
                    104: begin tx_data <= "X"; tx_start <= 1; state <= 50; end
                    50: begin tx_data <= to_hex(hum_int/10); tx_start <= 1; state <= 105; end
                    105: begin tx_data <= "X"; tx_start <= 1; state <= 51; end
                    51: begin tx_data <= to_hex(hum_int%10); tx_start <= 1; state <= 106; end
                    106: begin tx_data <= "X"; tx_start <= 1; state <= 52; end
                    52: begin tx_data <= "."; tx_start <= 1; state <= 107; end
                    107: begin tx_data <= "X"; tx_start <= 1; state <= 53; end
                    53: begin tx_data <= to_hex(hum_decimal); tx_start <= 1; state <= 108; end
                    108: begin tx_data <= "X"; tx_start <= 1; state <= 54; end
                    54: begin tx_data <= "-"; tx_start <= 1; state <= 109; end 
                    109: begin tx_data <= "X"; tx_start <= 1; state <= 55; end
                    55: begin tx_data <= "#"; tx_start <= 1; state <= 110; end
                    110: begin tx_data <= "X"; tx_start <= 1; state <= S_NL2; end
                    S_NL2: begin tx_data <= 8'h0A; tx_start <= 1; state <= 113; end
                    113: begin tx_data <= 0; state <= IDLE; end

                    114: begin tx_data <= "E" ; tx_start <= 1; state <= 115; end
                    115: begin tx_data <= "X" ; tx_start <= 1; state <= 116; end
                    116: begin tx_data <= "N" ; tx_start <= 1; state <= 117; end
                    117: begin tx_data <= "X" ; tx_start <= 1; state <= 118; end
                    118: begin tx_data <= "D" ; tx_start <= 1; state <= 119; end
                    119: begin tx_data <= "X" ; tx_start <= 1; state <= 120; end
                    120: begin tx_data <= "-" ; tx_start <= 1; state <= 121; end
                    121: begin tx_data <= "X" ; tx_start <= 1; state <= 122; end
                    122: begin tx_data <= "#" ; tx_start <= 1; state <= 123; end
                    123: begin tx_data <= "X" ; tx_start <= 1; state <= 124; end
                    124: begin tx_start <= 0; state <= 124; end

                    
                    
                    
                    // ---------- END ----------
                   /* 70: begin tx_data <= "E"; tx_start <= 1; state <= 71; end
                    71: begin tx_data <= "N"; tx_start <= 1; state <= 72; end 
                    72: begin tx_data <= "D"; tx_start <= 1; state <= 73; end
                    73: begin tx_data <= "-"; tx_start <= 1; state <= 74; end 
                    74: begin tx_data <= "#"; tx_start <= 1; state <= 75; end 
                    75: begin tx_done <= 1; state <= IDLE; end */
                    
                endcase

                 // slow down output (~2ms)
            end
        end
    end

always @(posedge clk) begin
    if(state == IDLE)
        delay_cnt <= 16'd100000;
    else if (delay_cnt > 0)
        delay_cnt <= delay_cnt - 1;
    
end

endmodule
