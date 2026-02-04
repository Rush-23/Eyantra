module bt_cmd_decoder (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  bt_byte,
    input  wire        bt_byte_valid,

    output reg  [2:0]  move_cmd,
    output reg         manual_mode
);

    localparam STOP    = 3'b000;
    localparam FORWARD = 3'b001;
    localparam LEFT    = 3'b010;
    localparam RIGHT   = 3'b011;
    localparam UTURN   = 3'b100;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            move_cmd    <= STOP;
            manual_mode <= 1'b0;
        end
        else if (bt_byte_valid) begin
            case (bt_byte)
                // Mode control (works anytime)
                "M", "m": manual_mode <= 1'b1;
                "A", "a": manual_mode <= 1'b0;
                
                // Motion control (only in manual mode)
                "F", "f": if (manual_mode) move_cmd <= FORWARD;
                "L", "l": if (manual_mode) move_cmd <= LEFT;
                "R", "r": if (manual_mode) move_cmd <= RIGHT;
                "U", "u": if (manual_mode) move_cmd <= UTURN;
                "S", "s": if (manual_mode) move_cmd <= STOP;
                
                default: ; // Ignore unknown bytes
            endcase
        end
    end

endmodule