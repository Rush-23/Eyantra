// Task 2C - MazeSolver Bot

module t2c_maze_explorer (
    input clk,
    input rst_n,
    input left, mid, right, // 0 - no wall, 1 - wall
    input wire move_done,
    output reg [2:0] move,
    input wire sense_valid,
    output wire  [3:0] dbg_col,
    output wire  [3:0] dbg_row,
    output wire [1:0] dbg_dir,
	output reg maze_ack,
    input wire ir,
    input wire maze_start,
    input wire [3:0] max_deadends,
    output reg maze_done,
    output reg [3:0] mpi_id,
    input wire [15:0] dist1,dist2,dist3
);

/*

| cmd | move  | meaning   |
|-----|-------|-----------|
| 000 | 0     | STOP      |
| 001 | 1     | FORWARD   |
| 010 | 2     | LEFT      |
| 011 | 3     | RIGHT     | 
| 100 | 4     | U_TURN    |

START POS   : 4,0
EXIT POS    : 4,8
DEADENDS    : 9

*/
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////
/*
# Team ID:          1024
# Theme:            MazeSolver Bot
# Author List:      Rushil V, Indiran T, Sathiya Naarayanan C, Charan Karthick A S
# Filename:         t2c_maze_explorer
# File Description: Simple maze explorer and solver with backtracking
# Global variables: N/A
*/

reg [4:0] d_left, d_fwd, d_right;
reg [4:0] best;

assign dbg_col = curr_col;
assign dbg_row = curr_row;
assign dbg_dir = dir;

// Parameters
parameter ROW = 9,               // number of rows in maze
          COL = 9,               // number of columns in maze
          ST_SIZE = ROW*COL;     // total number of cells


// Wires
wire update_freeze;              // prevents incrementing visited count while exploring further
wire solve_maze;                 // indicates exploration phase is complete
wire [1:0] open_paths;           // number of open directions at current cell
reg [3:0] dead_count;
reg [3:0] full_dead_count;


// Registers

// visited[col][row]: 2-bit visit state for each cell
// 0 = unvisited, 1 = once, 2 = twice/dead, 3 = junction
reg [1:0] visited [0:COL-1][0:ROW-1];

// Current bot position (row, col)
reg [3:0] curr_row, curr_col;

// Next bot position (row, col) to move to
reg [3:0] next_row, next_col;
wire [3:0] next_row_wire, next_col_wire;

// Current facing direction: 0 = North, 1 = East, 2 = South, 3 = West
reg [1:0] dir;
reg [1:0] next_dir;
// FSM states
localparam IDLE      = 0,        // initial setup
           WAIT      = 1,        // wait / decision state
           EXPLORE   = 2,        // exploration phase
           BACKTRACK = 3,
           WAIT_COMPLETE = 4,       // path back to goal
           HALT = 5;
reg [2:0] state;                 // current FSM state

// Deadend counter for exploration phase
//reg [3:0] dead_count;            // number of deadends discovered

// Flag to indicate switching to BACKTRACK
reg doneflag;                    // once set, stay in BACKTRACK
reg sensors_ready;
reg[24:0] halt_counter;




// Constants

// Goal cell coordinates
localparam goal_row = 0,         // goal row index
           goal_col = 4;         // goal column index


// Continuous Assignments

// freeze updates at goal cell once its visited count > 1
assign update_freeze = (visited[4][0] > 1);

// compute total open paths from sensor inputs
assign open_paths = (!left + !mid + !right);

// signal that maze exploration is done (all 9 deadends found & at a junction)
assign solve_maze = (full_dead_count >= 8 && open_paths > 1);

//integer to be used in for loop
integer i,j;

reg exploreflag;

// Main Sequential Process: FSM, movement, and marking

always @(posedge clk or negedge rst_n) begin

    // Reset section
    if (!rst_n) begin
        move      <= 3'b000;     // stop on reset
        //dir       <= 2'd0;       // default facing North
        state     <= IDLE;       // go to IDLE state
        dead_count <= 0;         // reset deadend counter
        doneflag  <= 0;          // clear done flag
        maze_done <= 0;
        exploreflag <= 0;
        //clear visited map
        for (i = 0; i < COL; i = i + 1)
            for (j = 0; j < ROW; j = j + 1)
                visited[i][j] <= 0;
    end 

    // Normal operation
    else begin
        case (state)

        // IDLE: initialize start position & wait
        IDLE: begin
          //  curr_col        <= 4;    // start column
          //  curr_row        <= 8;    // start row
            visited[4][8]   <= 0;    // clear start cell visited state
            move            <= 3'b000;
            mpi_id <= 4'b0;
            next_row <= 8;
            next_col <= 4;
            next_dir <= 0;
            if(maze_start)
                state           <= WAIT; // next go to WAIT
        end

        // WAIT: choose between EXPLORE and BACKTRACK
        WAIT: begin
            if(!sense_valid && !maze_done)
                state <= WAIT;
            else if (!doneflag)
                state <= solve_maze ? BACKTRACK : EXPLORE;  // switch to BACKTRACK when exploration done
            else
                state <= BACKTRACK;                         // once doneflag set, always BACKTRACK
        end

        // EXPLORE:exploration with left-hand rule
        EXPLORE: begin
            // default next position = stay put
            exploreflag <= 1;
            //next_row = curr_row;
            //next_col = curr_col;
			maze_ack        <= 1'b0;

            // Left-hand rule: LEFT → FORWARD → RIGHT, else U_TURN
            case (dir)
                // Facing North
                2'd0: begin
                    if (!left) begin
                        move     <= 3'b010;           // turn LEFT
                        next_dir      <= (dir + 3) & 2'b11;
                        next_col <= curr_col - 1;
                        next_row <= curr_row;
                    end 
                    else if (!mid && (curr_row > 0)) begin
                        move     <= 3'b001;           // go FORWARD
                        next_dir      <= dir;
                        next_row <= curr_row - 1;
                        next_col <= curr_col;
                    end 
                    else if (!right) begin
                        move     <= 3'b011;           // turn RIGHT
                        next_dir      <= (dir + 1) & 2'b11;
                        next_col <= curr_col + 1;
                        next_row <= curr_row;
                    end 
                    else if (left && mid && right && ir) begin
                        move     <= 3'b100;           // U_TURN
                        next_dir      <= (dir + 2) & 2'b11;
                        next_col <= curr_col;
                        next_row <= curr_row + 1;
                        mpi_id <= mpi_id + 1;
                        dead_count <= dead_count + 1; // count deadend
                    end
                    else if (left && mid && right) begin
                        move     <= 3'b101;           // U_TURN
                        next_dir      <= (dir + 2) & 2'b11;
                        next_col <= curr_col;
                        next_row <= curr_row + 1;
                        mpi_id <= mpi_id + 1;
                        full_dead_count <= full_dead_count + 1; // count deadend
                    end
                end

                // Facing East
                2'd1: begin
                    if (!left && (curr_row > 0)) begin
                        move     <= 3'b010;           // turn LEFT
                        next_dir      <= (dir + 3) & 2'b11;
                        next_row <= curr_row - 1;
                        next_col <= curr_col;
                    end 
                    else if (!mid) begin
                        move     <= 3'b001;           // go FORWARD
                        next_dir      <= dir;
                        next_col <= curr_col + 1;
                        next_row <= curr_row;
                    end 
                    else if (!right) begin
                        move     <= 3'b011;           // turn RIGHT
                        next_dir      <= (dir + 1) & 2'b11;
                        next_row <= curr_row + 1;
                        next_col <= curr_col;
                    end 
                    else if (left && mid && right && ir) begin
                        move     <= 3'b100;           // U_TURN
                        next_dir      <= (dir + 2) & 2'b11;
                        next_col <= curr_col - 1;
                        next_row <= curr_row;
                        mpi_id <= mpi_id + 1;
                        dead_count <= dead_count + 1; // count deadend
                    end
                    else if (left && mid && right) begin
                        move     <= 3'b101;           // U_TURN
                        next_dir      <= (dir + 2) & 2'b11;
                        next_col <= curr_col - 1;
                        next_row <= curr_row;
                        mpi_id <= mpi_id + 1;
                        full_dead_count <= full_dead_count + 1; // count deadend
                    end
                end

                // Facing South
                2'd2: begin
                    if (!left) begin
                        move     <= 3'b010;           // turn LEFT
                        next_dir      <= (dir + 3) & 2'b11;
                        next_col <= curr_col + 1;
                        next_row <= curr_row;
                    end 
                    else if (!mid) begin
                        move     <= 3'b001;           // go FORWARD
                        next_dir      <= dir;
                        next_row <= curr_row + 1;
                        next_col <= curr_col;
                    end 
                    else if (!right) begin
                        move     <= 3'b011;           // turn RIGHT
                        next_dir      <= (dir + 1) & 2'b11;
                        next_col <= curr_col - 1;
                        next_row <= curr_row;
                    end 
                    else if (left && mid && right && ir) begin
                        move     <= 3'b100;           // U_TURN
                        next_dir      <= (dir + 2) & 2'b11;
                        next_col <= curr_col;
                        mpi_id <= mpi_id + 1;
                        next_row <= curr_row - 1;
                        dead_count <= dead_count + 1; // count deadend
                    end
                    else if (left && mid && right) begin
                        move     <= 3'b101;           // U_TURN
                        next_dir      <= (dir + 2) & 2'b11;
                        next_col <= curr_col;
                        next_row <= curr_row - 1;
                        mpi_id <= mpi_id + 1;
                        full_dead_count <= full_dead_count + 1; // count deadend
                    end
                    
                end

                // Facing West
                2'd3: begin
                    if (!left) begin
                        move     <= 3'b010;           // turn LEFT
                        next_dir      <= (dir + 3) & 2'b11;
                        next_row <= curr_row + 1;
                        next_col <= curr_col;
                    end 
                    else if (!mid) begin
                        move     <= 3'b001;           // go FORWARD
                        next_dir      <= dir;
                        next_col <= curr_col - 1;
                        next_row <= curr_row;
                    end 
                    else if (!right && (curr_row > 0)) begin
                        move     <= 3'b011;           // turn RIGHT
                        next_dir      <= (dir + 1) & 2'b11;
                        next_row <= curr_row - 1;
                        next_col <= curr_col;
                    end 
                    else if (left && mid && right && ir) begin
                        move     <= 3'b100;           // U_TURN
                        next_dir      <= (dir + 2) & 2'b11;
                        next_col <= curr_col + 1;
                        next_row <= curr_row;
                        mpi_id <= mpi_id + 1;
                        dead_count <= dead_count + 1; // count deadend
                    end
                    else if (left && mid && right) begin
                        move     <= 3'b101;           // U_TURN
                        next_dir      <= (dir + 2) & 2'b11;
                        next_col <= curr_col + 1;
                        next_row <= curr_row;
                        mpi_id <= mpi_id + 1;
                        full_dead_count <= full_dead_count + 1; // count deadend
                end
                end
            endcase
				state <= WAIT_COMPLETE;
        end 

       
        // BACKTRACK: follow marked path with value 1 or 3 to go back to goal
       BACKTRACK: begin
            doneflag <= 1;  // latch that we are in backtrack mode
            //dbg_dir <= 2'b01;
           // next_row = curr_row;
           // next_col = curr_col;


            // At goal: emit final move based on facing direction
            if (curr_col == goal_col && curr_row == goal_row) begin    
                case (dir)
                    2'd0: move <= 3'b001; // facing North: FORWARD
                    2'd1: move <= 3'b010; // facing East : LEFT
                    2'd3: move <= 3'b011; // facing West : RIGHT
                    default: move <= 3'b100; // otherwise U_TURN
                endcase
            end
            // Not at goal: follow cells with visited == 1 or 3
            else begin
                // -------- Choose based on best with YOUR tie-break --------
                // priority: RIGHT > FORWARD > LEFT
                 case (dir)
                    // Facing North
                    2'd0: begin
                        if (!right && ((visited[curr_col+1][curr_row] == 1) || (visited[curr_col+1][curr_row] == 3))) begin
                            move     <= 3'b011;
                            next_dir      <= (dir + 1) & 2'b11;
                            next_col = curr_col + 1; next_row = curr_row;
                        end
                        else if (!left && ((visited[curr_col-1][curr_row] == 1) || (visited[curr_col-1][curr_row] == 3))) begin
                            move     <= 3'b010;
                            next_dir      <= (dir + 3) & 2'b11;
                            next_col = curr_col - 1; next_row = curr_row;
                        end 
                        else if (!mid && ((visited[curr_col][curr_row-1] == 1) || (visited[curr_col][curr_row-1] == 3))) begin
                            move     <= 3'b001;
                            next_dir      <= dir;
                            next_row = curr_row - 1; next_col = curr_col;
                        end 
                        else begin
                            move     <= 3'b100;
                            next_dir      <= (dir + 2) & 2'b11;
                            next_col = curr_col; next_row = curr_row + 1;
                        end
                    end

                    // Facing East
                    2'd1: begin
                        if (!left && ((visited[curr_col][curr_row-1] == 1) || (visited[curr_col][curr_row-1] == 3))) begin
                            move     <= 3'b010;
                            next_dir      <= (dir + 3) & 2'b11;
                            next_row = curr_row - 1; next_col = curr_col;
                        end 
                        else if (!mid && ((visited[curr_col+1][curr_row] == 1) || (visited[curr_col+1][curr_row] == 3))) begin
                            move     <= 3'b001;
                            next_dir      <= dir;
                            next_col = curr_col + 1; next_row = curr_row;
                        end 
                        else if (!right && ((visited[curr_col][curr_row+1] == 1) || (visited[curr_col][curr_row+1] == 3))) begin
                            move     <= 3'b011;
                            next_dir      <= (dir + 1) & 2'b11;
                            next_row = curr_row + 1; next_col = curr_col;
                        end 
                        else if (left && mid && right) begin
                            move     <= 3'b100;
                            next_dir      <= (dir + 2) & 2'b11;
                            next_col = curr_col - 1; next_row = curr_row;
                        end
                    end

                    // Facing South
                    2'd2: begin
                        if (!left && ((visited[curr_col+1][curr_row] == 1) || (visited[curr_col+1][curr_row] == 3))) begin
                            move     <= 3'b010;
                            next_dir      <= (dir + 3) & 2'b11;
                            next_col = curr_col + 1; next_row = curr_row;
                        end 
                        else if (!mid && ((visited[curr_col][curr_row+1] == 1) || (visited[curr_col][curr_row+1] == 3))) begin
                            move     <= 3'b001;
                            next_dir      <= dir;
                            next_row = curr_row + 1; next_col = curr_col;
                        end 
                        else if (!right && ((visited[curr_col-1][curr_row] == 1) || (visited[curr_col-1][curr_row] == 3))) begin
                            move     <= 3'b011;
                            next_dir      <= (dir + 1) & 2'b11;
                            next_col = curr_col - 1; next_row = curr_row;
                        end 
                        else begin
                            move     <= 3'b100;
                            next_dir      <= (dir + 2) & 2'b11;
                            next_col = curr_col; next_row = curr_row - 1;
                        end
                    end

                    // Facing West
                    2'd3: begin
                        if (!left && ((visited[curr_col][curr_row+1] == 1) || (visited[curr_col][curr_row+1] == 3))) begin
                            move     <= 3'b010;
                            next_dir      <= (dir + 3) & 2'b11;
                            next_row = curr_row + 1; next_col = curr_col;
                        end 
                        else if (!mid && ((visited[curr_col-1][curr_row] == 1) || (visited[curr_col-1][curr_row] == 3))) begin
                            move     <= 3'b001;
                            next_dir      <= dir;
                            next_col = curr_col - 1; next_row = curr_row;
                        end 
                        else if (!right && ((visited[curr_col][curr_row-1] == 1) || (visited[curr_col][curr_row-1] == 3))) begin
                            move     <= 3'b011;
                            next_dir      <= (dir + 1) & 2'b11;
                            next_row = curr_row - 1; next_col = curr_col;
                        end 
                        else begin
                            move     <= 3'b100;
                            next_dir      <= (dir + 2) & 2'b11;
                            next_col = curr_col + 1; next_row = curr_row;
                        end
                    end
                endcase
            end



            // commit new position and return to WAIT
            state    <= WAIT_COMPLETE;
        end

        WAIT_COMPLETE: begin
             if (move_done) begin

                //move     <= 3'b000;
                
				if (open_paths == 0)
                     visited[curr_col][curr_row] <= 2;  // deadend
                 else if (open_paths == 1 && !update_freeze)
                     visited[curr_col][curr_row] <= visited[curr_col][curr_row] + 1;
                 else if (open_paths > 1)
                     visited[curr_col][curr_row] <= 3;

                 if (curr_row == goal_row && curr_col == goal_col)
                    visited[curr_col][curr_row] <= 3;

                if(dist2 > 500 && dist1 > 800 && dist3 > 500) begin
                    maze_done <= 1'b1;
                    state <= IDLE;
                end

                if(halt_counter >= 1_000_000) begin
                    if (!doneflag)
                        state <= solve_maze ? BACKTRACK : EXPLORE;  // switch to BACKTRACK when exploration done
                    else
                        state <= BACKTRACK;
                    maze_ack <= 1'b1;
                end

                
    end
end



        // default

        default: begin
            state <= IDLE;
        end

        endcase
    end
end 
//assign dbg_dir = dir;
always@(posedge clk) begin
    if(state == WAIT_COMPLETE && move_done) halt_counter <= halt_counter + 1;
    else halt_counter <= 0;
end


always@(posedge clk) begin
    if(state == IDLE) begin
        curr_row <= 8;
        curr_col <= 4;
        dir <= 0;
    end
    else if(state == WAIT_COMPLETE) begin

    curr_row <= next_row;
    curr_col <= next_col;
    dir      <= next_dir;
    end
end

/*always@(*) begin
                d_left  = 5'd31;
                d_fwd   = 5'd31;
                d_right = 5'd31;

                // ---------- LEFT candidate ----------
                case (dir)
                    2'd0: if (!left)       d_left = manhattan(curr_row, curr_col - 1); // W
                    2'd1: if (!left)       d_left = manhattan(curr_row - 1, curr_col); // N
                    2'd2: if (!left)   d_left = manhattan(curr_row, curr_col + 1); // E
                    2'd3: if (!left)   d_left = manhattan(curr_row + 1, curr_col); // S
                endcase

                // ---------- FORWARD candidate ----------
                case (dir)
                    2'd0: if (!mid)         d_fwd = manhattan(curr_row - 1, curr_col); // N
                    2'd1: if (!mid)     d_fwd = manhattan(curr_row, curr_col + 1); // E
                    2'd2: if (!mid)     d_fwd = manhattan(curr_row + 1, curr_col); // S
                    2'd3: if (!mid)         d_fwd = manhattan(curr_row, curr_col - 1); // W
                endcase

                // ---------- RIGHT candidate ----------
                case (dir)
                    2'd0: if (!right)   d_right = manhattan(curr_row, curr_col + 1); // E
                    2'd1: if (!right)   d_right = manhattan(curr_row + 1, curr_col); // S
                    2'd2: if (!right)       d_right = manhattan(curr_row, curr_col - 1); // W
                    2'd3: if (!right)       d_right = manhattan(curr_row - 1, curr_col); // N
                endcase

                // best distance
                best = d_left;
                if (d_fwd   < best) best = d_fwd;
                if (d_right < best) best = d_right;
end
//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////
*/
endmodule