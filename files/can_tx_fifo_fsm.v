// ---------------------------------------------------------------------
// can_tx_fifo_fsm.v
// Pops one frame at a time from this channel's TX FIFO and hands it to
// the register-level TX FSM.
// Frame in the FIFO (32-bit words):  ID, DLC, N, data[0..N-1]   (N <= 16)
// The TX router guarantees N <= 16 and that exactly N data words follow.
// ---------------------------------------------------------------------
module can_tx_fifo_fsm (
    input  wire         aclk,
    input  wire         aresetn,

    input  wire [31:0]  fifo_dout,
    input  wire         fifo_empty,
    output wire         fifo_rd_en,

    output wire         tx_valid,
    input  wire         tx_ready,
    output reg  [31:0]  tx_id,
    output reg  [31:0]  tx_dlc,
    output reg  [4:0]   tx_nwords,
    output reg  [511:0] tx_data,
    input  wire         tx_done,
    input  wire         tx_error,
    input  wire [2:0]   tx_err_code,

    output reg  [15:0]  frames_sent,
    output reg  [15:0]  frames_err,
    output reg  [2:0]   last_err
);
    localparam [2:0] F_ID = 3'd0, F_DLC = 3'd1, F_NW = 3'd2,
                     F_DATA = 3'd3, F_SEND = 3'd4, F_WAIT = 3'd5;

    reg [2:0] st;
    reg [4:0] k;

    // each of the four read states consumes exactly one word when not empty
    assign fifo_rd_en = (st <= F_DATA) && !fifo_empty;
    assign tx_valid   = (st == F_SEND);

    always @(posedge aclk) begin
        if (!aresetn) begin
            st <= F_ID; k <= 5'd0;
            tx_id <= 32'd0; tx_dlc <= 32'd0; tx_nwords <= 5'd0; tx_data <= 512'd0;
            frames_sent <= 16'd0; frames_err <= 16'd0; last_err <= 3'd0;
        end else begin
            case (st)
            F_ID: if (!fifo_empty) begin
                tx_id <= fifo_dout; tx_data <= 512'd0; st <= F_DLC;
            end

            F_DLC: if (!fifo_empty) begin
                tx_dlc <= fifo_dout; st <= F_NW;
            end

            F_NW: if (!fifo_empty) begin
                tx_nwords <= fifo_dout[4:0];
                k  <= 5'd0;
                st <= (fifo_dout[4:0] == 5'd0) ? F_SEND : F_DATA;
            end

            F_DATA: if (!fifo_empty) begin
                tx_data[{k, 5'b00000} +: 32] <= fifo_dout;
                if (k == tx_nwords - 5'd1) st <= F_SEND;
                else k <= k + 1'b1;
            end

            // can_tx_fsm latches the frame on the edge where tx_valid & tx_ready
            F_SEND: if (tx_ready) st <= F_WAIT;

            F_WAIT: begin
                if (tx_done) begin
                    frames_sent <= frames_sent + 1'b1; st <= F_ID;
                end else if (tx_error) begin
                    frames_err <= frames_err + 1'b1;
                    last_err   <= tx_err_code;
                    st <= F_ID;
                end
            end

            default: st <= F_ID;
            endcase
        end
    end
endmodule
