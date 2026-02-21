module top(
    input  wire        clk_50M,
    input  wire        reset,

    input  wire        echo1,
    input  wire        echo2,
    input  wire        echo3,

    output wire        trig1,
    output wire        trig2,
    output wire        trig3,

    output wire [15:0] dist1,
    output wire [15:0] dist2,
    output wire [15:0] dist3,

    input wire encL_A,
    input wire encL_B,
    input wire encR_A,
    input wire encR_B,
    inout dht_input,

    input wire IR_SENSOR_PIN1,
    input wire IR_SENSOR_PIN2,

    output servo_pwm,

    input  wire bt_rx,
    input  wire dout,
    output wire bt_tx,
    output adc_cs_n, adc_sck, din, 

    output reg         op1_latched,
    output reg         op2_latched,
    output reg         op3_latched,
    output wire [7:0] led,

    output wire [2:0] movef, rx_complete,
    
    output IN1,
    output IN2,
    output IN3,
    output IN4,
    output EN_A, 
    output EN_B
    );

    // ================================
    // Sensor Selection FSM
    // ================================
    localparam S1 = 2'd0,
               S2 = 2'd1,
               S3 = 2'd2;

    reg [1:0] sensor_sel;
    reg [21:0] slot_counter;
    reg [2:0] movefinal;
    
    assign led[0] = maze_start;


    // 40 ms silence + ~25 ms active
    // 50 MHz → 3,250,000 cycles ≈ 65 ms per sensor
    wire op1;
    wire op2;
    wire op3;

    always @(posedge clk_50M or negedge reset) begin
    if (!reset) begin
        op1_latched <= 1'b0;
        op2_latched <= 1'b0;
        op3_latched <= 1'b0;
    end else begin
        if (en1) op1_latched <= op1;
        if (en2) op2_latched <= op2;
        if (en3) op3_latched <= op3;
    end
end

    reg op1_hold, op2_hold, op3_hold;
/* 
always @(posedge clk_50M or negedge reset) begin
    if (!reset) begin
        op1_hold <= 0;
        op2_hold <= 0;
        op3_hold <= 0;
    end else begin
        if (en1) op1_hold <= op1_latched;
        if (en2) op2_hold <= op2_latched;
        if (en3) op3_hold <= op3_latched;
    end
end
*/

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            sensor_sel   <= S1;
            slot_counter <= 0;
        end else begin
            if (slot_counter >= 22'd5_000_000) 
            begin
                slot_counter <= 0;
                sensor_sel <= (sensor_sel == S3) ? S1 : sensor_sel + 1'b1;
            end else 
                slot_counter <= slot_counter + 1'b1;
        end
    end

   reg left_snap, mid_snap, right_snap;
    //block to latch the value once all three sensors measure (one complete cycle)
   always @(posedge clk_50M or negedge reset) begin
    if (!reset) begin
        left_snap  <= 1'b0;
        mid_snap   <= 1'b0;
        right_snap <= 1'b0;
    end else if (sensor_frame_done) begin
        left_snap  <= op3_hold;
        mid_snap   <= op2_hold;
        right_snap <= op1_hold;
    end
    end 

// ================================
// Bluetooth (BLE) logic
// ================================
wire [7:0] rx_msg;

reg  [7:0] bt_byte;
reg        bt_byte_valid;
reg        rx_complete_d;

wire [2:0] bt_move;
wire       manual_mode;

// UART Receiver
uart_rx uart (
    .clk(clk_50M),
    .rx(bt_rx),
    .rx_msg(rx_msg),
    .rst_n(reset),
    .rx_complete(rx_complete)
);

// rx_complete edge detect
reg [7:0] bt_byte_d;
reg       bt_byte_valid_d;

reg reset_n;

localparam RESET_CYCLES = 500_000; // 10 ms at 50 MHz

reg [18:0] reset_counter;  // enough bits for 500,000

always @(posedge clk_50M) begin
    if (reset_counter < RESET_CYCLES) begin
            reset_counter <= reset_counter + 1'b1;
            reset_n <= 1'b0;       // hold reset active
    end else begin
            reset_n <= 1'b1;       // release reset
    end
 end



always @(posedge clk_50M or negedge reset_n) begin
    if (!reset_n) begin
        bt_byte         <= 8'd0;
        bt_byte_d       <= 8'd0;
        bt_byte_valid   <= 1'b0;
        bt_byte_valid_d <= 1'b0;
        rx_complete_d   <= 1'b0;
    end else begin
        rx_complete_d <= rx_complete;

        // Capture UART byte on rx_complete rising edge
        bt_byte_d       <= rx_msg;
        if (rx_complete && !rx_complete_d) begin
            bt_byte_valid_d <= 1'b1;
        end else begin
            bt_byte_valid_d <= 1'b0;
        end

        // One-cycle delayed, clean interface to decoder
        bt_byte       <= bt_byte_d;
        bt_byte_valid <= bt_byte_valid_d;
    end
end



// BLE command decoder
bt_cmd_decoder bt (
    .clk(clk_50M),
    .rst_n(reset_n),
    .bt_byte(bt_byte),
    .bt_byte_valid(bt_byte_valid),
    .code_go(maze_start),
    .ded_count(dead_count_max)
);

wire maze_start;
wire [3:0] dead_count_max;
// Arbitration
    wire [2:0] final_move;
    assign final_move = manual_mode ? bt_move : movef;
   // assign led[3] = btx_done;

    // ================================
    // Enable signals
    // ================================
    wire en1 = (sensor_sel == S1);
    wire en2 = (sensor_sel == S2);
    wire en3 = (sensor_sel == S3);
    wire sensor_frame_done = (sensor_sel == S3 && slot_counter == 22'd4_999_998);

    // ================================
    // Ultrasonic instances
    // ================================
    t1b_ultrasonic u1 (
        .clk_50M(clk_50M),
        .reset  (reset_n),
        .enable (en1),
        .echo_rx(echo1),
        .trig   (trig1),
        .op     (op1),
        .distance_out(dist1)
    );

    t1b_ultrasonic u2 (
        .clk_50M(clk_50M),
        .reset  (reset_n),
        .enable (en2),
        .echo_rx(echo2),
        .trig   (trig2),
        .op     (op2),
        .distance_out(dist2)
    );

    t1b_ultrasonic u3 (
        .clk_50M(clk_50M),
        .reset  (reset_n),
        .enable (en3),
        .echo_rx(echo3),
        .trig   (trig3),
        .op     (op3),
        .distance_out(dist3)
    );

    wire [3:0] row,col;
    wire [1:0] dir;
	wire maze_ack;

    wire [3:0] df,ef;

    t2c_maze_explorer m1 (
        .clk(clk_50M),
        .rst_n(reset_n),
        .left(op3_latched),
        .mid(op2_latched),
        .right(op1_latched),
        .move(movef),
        .move_done(move_done_wire),
        .sense_valid(sensor_frame_done),
        .dbg_col(row),
        .dbg_row(col),
        .dbg_dir(dir),
		.maze_ack(maze_ack),
        .ir(ir_true),
        .maze_start(maze_start),
        .max_deadends(dead_count_max),        
        .maze_done(maze_done),
        .mpi_id(dead_count),
        .dist1(dist1),
        .dist2(dist2),
        .dist3(dist3)
    );

    //assign led[7:6] = msg_type;
    //assign led[5]   = moisture_done;
    wire [3:0] dead_count;
 
    // ================================
    // Maze solver → motion controller
    // ================================
    wire [2:0] motor_cmd;
    wire       motor_enable;
    wire       move_done_wire;

    controller cont (
        .clk(clk_50M),
        .reset(reset),
        .move(final_move),

        .encL_A(encL_A),
        .encL_B(encL_B),
        .encR_A(encR_A),
        .encR_B(encR_B),

        .dist1(dist1),
        .dist2(dist2),
        .dist3(dist3),

        .to_motordriver(motor_cmd),
        .enable(motor_enable),
        .move_done(move_done_wire),
		.maze_ack(maze_ack),
        .mpi_start(mpi_start),
        .ir(ir_true),
        .mpi_done(mpi_done),
        .uturn_done(uturn_done),
        .maze_done(maze_done)
    );

    // ================================
    // Motor driver
    // ================================
    motor mot(
        .clk_50(clk_50M),
        .rst(reset),

        .move_cmd(motor_cmd),
        .enable(motor_enable),

        .l_in1(IN1),
        .l_in2(IN2),
        .r_in1(IN3),
        .r_in2(IN4),
        .en_l(EN_A), 
        .en_r(EN_B)
    );

    wire ir_pulse1,ir_pulse2;
    wand ir_true;
    assign ir_true = ir_pulse1;
    assign ir_true = ir_pulse2; 

    ir ir_module1 (
        .clk(clk_50M),
        .rst_n(reset),
        .ir_in(IR_SENSOR_PIN1),
        .deadend_pulse(ir_pulse1)
    );

    ir ir_module2 (
        .clk(clk_50M),
        .rst_n(reset),
        .ir_in(IR_SENSOR_PIN2),
        .deadend_pulse(ir_pulse2)
    );

    wire servo_done,servo_start,servo_release;
   // assign servo_start = ir_true;

    servo servo(
        .clk_50M(clk_50M),
        .reset(reset),
        .servo_start(servo_start),
        .servo_done(servo_done),
		.servo_pwm(servo_pwm),
        .servo_release(servo_release),
        .dipped(dipped)
    );
    
    wire dipped;
    wire data_valid;
    wire [7:0] temp_int,temp_decimal,hum_int,hum_decimal;

    dht dht (
        .clk_50M(clk_50M),
        .reset(reset),

        .sensor(dht_input),
        .T_integral(temp_int),
        .T_decimal(temp_decimal),
        .RH_integral(hum_int),
        .RH_decimal(hum_decimal),
        .data_valid(data_valid)
    );

    wire mpi_start,maze_done,mpi_done;

    wire uturn_done;
    mpicontroller mpi (
        .clk(clk_50M),
        .reset(reset),

        .maze_done_pulse(maze_done),
        .mpi_start(mpi_start),
        .servo_done(servo_done),
        .tx_done(btx_done),
        .dht_done(),
        .moisture_status(moisture_status),
        .moisture_done(mdone),
        .servo_release(servo_release),
        .servo_start(servo_start),
        .dht_start(),
        .moisture_start(moisture_start),
        .send_start(send_start),
        .msg_type(msg_type),
        .mpi_done(mpi_done),
        .dipped(dipped),
        .uturn_done(uturn_done)
    ); 

    
    wire moisture_start;

    wire [1:0] msg_type;
    wire btx_done,send_start;

    wire uart_tx_done;
    wire uart_busy;
    wire [7:0] tx_data;
    wire tx_start;
    wire bt_moist_done;
    assign bt_moist_done = moist_bt_done;

    uart_tx uart1 (
        .clk(clk_50M), 
        .rst(reset),
        .parity_type(1'b0),// unused
        .tx_start(tx_start),
        .data(tx_data),
        .tx(bt_tx), 
        .tx_done(uart_tx_done),
        .busy(uart_busy)
    );

    bt_tx_control bt_txm (
        .clk(clk_50M),
        .rst(reset),
        .uart_busy(uart_busy),
        .moist_done(servo_release),
        .tx_start(tx_start),
        .tx_data(tx_data),
        .msg_type(msg_type),
        .btx_done(btx_done),
        .send_start(send_start),
        .mord(mord),
        .dead_count(dead_count),
        .temp_int(temp_int),
        .temp_decimal(temp_decimal),
        .hum_int(hum_int),
        .hum_decimal(hum_decimal),
        .maze_done(maze_done),
        .row(row),
        .col(col),
        .dir(dir)
    );

    wire moisture_status;
    wire [11:0] moist_value;
    //assign led[7] = mpi_done;
    reg moisture_done;
    moisture_sensor ms (
        .clk50(clk_50M),
        .dout(dout),
        .adc_cs_n(adc_cs_n),
        .din(din),
        .adc_sck(adc_sck),
        .d_out_ch0(moist_value),
        .led_ind()
    );

    reg mord = 0;
    always @(posedge clk_50M or negedge reset) begin
    if (!reset)
        mord <= 0;
    else if (moist_count == 10_000_000) begin
        if (dead_count == 1)
            mord <= 1;
        else if (dead_count == 3)
            mord <= 0;
        else if (dead_count == 4)
            mord <= 1;
    end
    end


    reg [31:0] moist_count = 32'd0;
    reg moist_bt_done;
    localparam MOIST_CYCLES  = 300_000_000;
    

    always @(posedge clk_50M or negedge reset) begin
    if (!reset) begin
        moist_count    <= 0;
        moisture_done  <= 0;
    end 
    else if (moisture_start) begin
        moist_count    <= 0;
        moisture_done  <= 0;
        //moist_bt_done  <= 0;
    end 
    else begin
        if (moist_count >= MOIST_CYCLES-1) begin
            moist_count   <= 0;
            moisture_done <= 1;   // 1-cycle pulse
            //moist_bt_done <= 1;
        end 
        else begin
            moist_count   <= moist_count + 1;
            moisture_done <= 0;
        end
        end
    end


    wire mdone;
    assign mdone = moisture_done;

endmodule





 
    

   