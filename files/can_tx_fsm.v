// ---------------------------------------------------------------------
// can_tx_fsm.v
// Register-level TX FSM for ONE CAN FD IP.
// Writes ID / DLC / data words into a TX buffer, sets the TRR bit, then
// polls TRR until the hardware clears it (frame sent).
//
// !! Offsets are from memory of PG223 - VERIFY against your IP version.
// !! If your IP does not clear TRR on completion, poll TXOK in ISR instead.
// ---------------------------------------------------------------------
module can_tx_fsm #(
    parameter [11:0] ADDR_TRR   = 12'h090,
    parameter [11:0] ADDR_TXMSG = 12'h100,      // TX buffer 0 ID register
    parameter integer TB_STRIDE = 72,           // 0x48 bytes per TX buffer
    parameter integer TX_BUF    = 0,            // which TX buffer to use
    parameter [15:0] TX_POLL_TIMEOUT = 16'hFFFF
)(
    input  wire         aclk,
    input  wire         aresetn,
    input  wire         link_up,                // cfg_done && !quiesce
    input  wire [31:0]  base_addr,

    input  wire         tx_valid,
    output wire         tx_ready,
    input  wire [31:0]  tx_id,                  // raw ID register word
    input  wire [31:0]  tx_dlc,                 // raw DLC register word
    input  wire [4:0]   tx_nwords,              // 32-bit data words, 0..16
    input  wire [511:0] tx_data,                // word 0 = bits [31:0]
    output reg          tx_done,                // 1-cycle pulse
    output reg          tx_error,               // 1-cycle pulse
    output reg  [2:0]   tx_err_code,            // 1 AXI TO, 2 AXI resp, 3 buffer busy, 4 TX timeout
    output wire         tx_idle,

    output reg          c_valid,
    output reg          c_we,
    output reg  [31:0]  c_addr,
    output reg  [31:0]  c_wdata,
    input  wire         rsp_valid,
    input  wire [31:0]  rsp_rdata,
    input  wire         rsp_err,
    input  wire [1:0]   rsp_code
);
    localparam [3:0] T_IDLE  = 4'd0,  T_CHK_I = 4'd1,  T_CHK_W = 4'd2,
                     T_WR_I  = 4'd3,  T_WR_W  = 4'd4,  T_REQ_I = 4'd5,
                     T_REQ_W = 4'd6,  T_POL_I = 4'd7,  T_POL_W = 4'd8,
                     T_DONE  = 4'd9,  T_ERR   = 4'd10;

    reg [3:0]   state;
    reg [31:0]  base_l, id_l, dlc_l;
    reg [4:0]   nw_l, k;
    reg [511:0] data_l;
    reg [15:0]  poll_cnt;

    assign tx_ready = (state == T_IDLE) && link_up;
    assign tx_idle  = (state == T_IDLE);

    wire [31:0] txb     = base_l + {20'd0, ADDR_TXMSG} + (TX_BUF * TB_STRIDE);
    wire [4:0]  widx    = k - 5'd2;
    wire [31:0] dw      = data_l[{widx, 5'b00000} +: 32];
    wire [2:0]  axi_err = (rsp_code == 2'd1) ? 3'd1 : 3'd2;

    // byte offset inside the TX buffer: 0 = ID, 4 = DLC, 8+4n = data word n
    wire [31:0] woff = (k == 5'd0) ? 32'd0 :
                       (k == 5'd1) ? 32'd4 :
                                     (32'd8 + {25'd0, widx, 2'b00});

    always @(posedge aclk) begin
        if (!aresetn) begin
            state <= T_IDLE; k <= 5'd0; nw_l <= 5'd0; poll_cnt <= 16'd0;
            base_l <= 32'd0; id_l <= 32'd0; dlc_l <= 32'd0; data_l <= 512'd0;
            tx_done <= 1'b0; tx_error <= 1'b0; tx_err_code <= 3'd0;
            c_valid <= 1'b0; c_we <= 1'b0; c_addr <= 32'd0; c_wdata <= 32'd0;
        end else begin
            tx_done  <= 1'b0;
            tx_error <= 1'b0;

            case (state)
            T_IDLE: if (tx_valid && link_up) begin
                base_l <= base_addr; id_l <= tx_id; dlc_l <= tx_dlc;
                nw_l   <= tx_nwords; data_l <= tx_data;
                state  <= T_CHK_I;
            end

            // buffer must be free (TRR bit clear)
            T_CHK_I: begin
                c_we <= 1'b0; c_addr <= base_l + {20'd0, ADDR_TRR};
                c_valid <= 1'b1; state <= T_CHK_W;
            end
            T_CHK_W: if (rsp_valid) begin
                c_valid <= 1'b0;
                if (rsp_err) begin tx_err_code <= axi_err; state <= T_ERR; end
                else if (rsp_rdata[TX_BUF]) begin tx_err_code <= 3'd3; state <= T_ERR; end
                else begin k <= 5'd0; state <= T_WR_I; end
            end

            // k = 0 ID, k = 1 DLC, k >= 2 data words
            T_WR_I: begin
                c_we    <= 1'b1;
                c_addr  <= txb + woff;
                c_wdata <= (k == 5'd0) ? id_l : (k == 5'd1) ? dlc_l : dw;
                c_valid <= 1'b1;
                state   <= T_WR_W;
            end
            T_WR_W: if (rsp_valid) begin
                c_valid <= 1'b0;
                if (rsp_err) begin tx_err_code <= axi_err; state <= T_ERR; end
                else if (k == nw_l + 5'd1) state <= T_REQ_I;
                else begin k <= k + 1'b1; state <= T_WR_I; end
            end

            // request transmission
            T_REQ_I: begin
                c_we <= 1'b1; c_addr <= base_l + {20'd0, ADDR_TRR};
                c_wdata <= (32'd1 << TX_BUF); c_valid <= 1'b1; state <= T_REQ_W;
            end
            T_REQ_W: if (rsp_valid) begin
                c_valid <= 1'b0;
                if (rsp_err) begin tx_err_code <= axi_err; state <= T_ERR; end
                else begin poll_cnt <= 16'd0; state <= T_POL_I; end
            end

            // wait for TRR bit to clear = frame sent
            T_POL_I: begin
                c_we <= 1'b0; c_addr <= base_l + {20'd0, ADDR_TRR};
                c_valid <= 1'b1; state <= T_POL_W;
            end
            T_POL_W: if (rsp_valid) begin
                c_valid <= 1'b0;
                if (rsp_err) begin tx_err_code <= axi_err; state <= T_ERR; end
                else if (!rsp_rdata[TX_BUF]) state <= T_DONE;
                else if (poll_cnt == TX_POLL_TIMEOUT) begin tx_err_code <= 3'd4; state <= T_ERR; end
                else begin poll_cnt <= poll_cnt + 1'b1; state <= T_POL_I; end
            end

            T_DONE: begin tx_done  <= 1'b1; state <= T_IDLE; end
            T_ERR:  begin tx_error <= 1'b1; c_valid <= 1'b0; state <= T_IDLE; end
            default: state <= T_ERR;
            endcase
        end
    end
endmodule
