// ---------------------------------------------------------------------
// can_rx_fifo_fsm.v
// Takes a received frame from the register-level RX FSM and pushes it into
// this channel's RX FIFO as 32-bit words:  ID, DLC, N, data[0..N-1]
// Stalls (never drops) while the FIFO is full.
// ---------------------------------------------------------------------
module can_rx_fifo_fsm (
    input  wire         aclk,
    input  wire         aresetn,

    input  wire         rx_valid,
    output wire         rx_ready,
    input  wire [31:0]  rx_id,
    input  wire [31:0]  rx_dlc,
    input  wire [4:0]   rx_nwords,
    input  wire [511:0] rx_data,

    output wire [31:0]  fifo_din,
    output wire         fifo_wr_en,
    input  wire         fifo_full
);
    localparam [1:0] K_IDLE = 2'd0, K_PUSH = 2'd1, K_ACK = 2'd2;

    reg [1:0] st;
    reg [4:0] k;

    wire [4:0] last = rx_nwords + 5'd2;          // index of the final word
    wire [4:0] didx = k - 5'd3;                  // data word index (k >= 3)

    assign fifo_din   = (k == 5'd0) ? rx_id  :
                        (k == 5'd1) ? rx_dlc :
                        (k == 5'd2) ? {27'd0, rx_nwords} :
                                      rx_data[{didx, 5'b00000} +: 32];
    assign fifo_wr_en = (st == K_PUSH) && !fifo_full;
    assign rx_ready   = (st == K_ACK);

    always @(posedge aclk) begin
        if (!aresetn) begin
            st <= K_IDLE; k <= 5'd0;
        end else begin
            case (st)
            K_IDLE: if (rx_valid) begin k <= 5'd0; st <= K_PUSH; end
            K_PUSH: if (!fifo_full) begin
                if (k == last) st <= K_ACK;
                else k <= k + 1'b1;
            end
            K_ACK:  begin st <= K_IDLE; k <= 5'd0; end
            default: st <= K_IDLE;
            endcase
        end
    end
endmodule
