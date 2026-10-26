// ---------------------------------------------------------------------
// can_rx_fsm.v
// Register-level RX FSM for ONE CAN FD IP (RX FIFO 0).
// Polls the FIFO fill level, reads one frame (ID, DLC, data words),
// advances the FIFO read index, then presents the frame on rx_*.
//
// !! Offsets / bit fields are from memory of PG223 - VERIFY them.
// ---------------------------------------------------------------------
module can_rx_fsm #(
    parameter [11:0] ADDR_FSR   = 12'h0E8,
    parameter [31:0] RXMSG_BASE = 32'h0000_2100,   // RX FIFO0 buffer 0 ID register (PG223: 0x2100, +0x48 per buffer)
    parameter integer RB_STRIDE = 72,              // 0x48 bytes per RX buffer
    parameter [15:0] RX_POLL_PERIOD = 16'd200,     // aclk cycles between FSR polls
    parameter integer RX_EN = 1                    // 0 = stay idle: never touch FSR / the RX buffers
)(
    input  wire         aclk,
    input  wire         aresetn,
    input  wire         link_up,
    input  wire [31:0]  base_addr,

    output reg          rx_valid,                  // held until rx_ready
    input  wire         rx_ready,
    output reg  [31:0]  rx_id,
    output reg  [31:0]  rx_dlc,
    output reg  [4:0]   rx_nwords,
    output reg  [511:0] rx_data,
    output reg          rx_error,                  // 1-cycle pulse
    output reg  [2:0]   rx_err_code,               // 1 AXI timeout, 2 AXI resp
    output wire         rx_idle,

    output reg          c_valid,
    output reg          c_we,
    output reg  [31:0]  c_addr,
    output reg  [31:0]  c_wdata,
    input  wire         rsp_valid,
    input  wire [31:0]  rsp_rdata,
    input  wire         rsp_err,
    input  wire [1:0]   rsp_code
);
    localparam [3:0] R_WAIT  = 4'd0, R_FSR_I = 4'd1, R_FSR_W = 4'd2,
                     R_RD_I  = 4'd3, R_RD_W  = 4'd4, R_IRI_I = 4'd5,
                     R_IRI_W = 4'd6, R_OUT   = 4'd7, R_ERR   = 4'd8;

    reg [3:0]  state;
    reg [31:0] base_l;
    reg [5:0]  ri;
    reg [4:0]  k, nw;
    reg [15:0] cnt;

    assign rx_idle = (state == R_WAIT) || (state == R_OUT);

    // DLC code -> number of 32-bit data words
    function [4:0] dlc2words;
        input [3:0] dlc;
        input       edl;
        begin
            if (!edl && dlc > 4'd8) dlc2words = 5'd2;
            else case (dlc)
                4'd0:                      dlc2words = 5'd0;
                4'd1, 4'd2, 4'd3, 4'd4:    dlc2words = 5'd1;
                4'd5, 4'd6, 4'd7, 4'd8:    dlc2words = 5'd2;
                4'd9:                      dlc2words = 5'd3;    // 12 B
                4'd10:                     dlc2words = 5'd4;    // 16 B
                4'd11:                     dlc2words = 5'd5;    // 20 B
                4'd12:                     dlc2words = 5'd6;    // 24 B
                4'd13:                     dlc2words = 5'd8;    // 32 B
                4'd14:                     dlc2words = 5'd12;   // 48 B
                default:                   dlc2words = 5'd16;   // 64 B
            endcase
        end
    endfunction

    wire [31:0] rxb    = base_l + RXMSG_BASE + ({26'd0, ri} * RB_STRIDE);
    wire [4:0]  widx   = k - 5'd2;
    wire [4:0]  nw_now = (k == 5'd1) ? dlc2words(rsp_rdata[31:28], rsp_rdata[27]) : nw;
    wire [2:0]  axi_err = (rsp_code == 2'd1) ? 3'd1 : 3'd2;

    wire [31:0] roff = (k == 5'd0) ? 32'd0 :
                       (k == 5'd1) ? 32'd4 :
                                     (32'd8 + {25'd0, widx, 2'b00});

    always @(posedge aclk) begin
        if (!aresetn) begin
            state <= R_WAIT; base_l <= 32'd0; ri <= 6'd0; k <= 5'd0; nw <= 5'd0; cnt <= 16'd0;
            rx_valid <= 1'b0; rx_id <= 32'd0; rx_dlc <= 32'd0; rx_nwords <= 5'd0; rx_data <= 512'd0;
            rx_error <= 1'b0; rx_err_code <= 3'd0;
            c_valid <= 1'b0; c_we <= 1'b0; c_addr <= 32'd0; c_wdata <= 32'd0;
        end else begin
            rx_error <= 1'b0;

            case (state)
            R_WAIT: begin
                if (!link_up || RX_EN == 0) cnt <= 16'd0;
                else if (cnt == RX_POLL_PERIOD) begin
                    cnt <= 16'd0; base_l <= base_addr; state <= R_FSR_I;
                end else cnt <= cnt + 1'b1;
            end

            // FSR: FL = [14:8] fill level, IRI = bit 7, RI = [5:0] read index
            R_FSR_I: begin
                c_we <= 1'b0; c_addr <= base_l + {20'd0, ADDR_FSR};
                c_valid <= 1'b1; state <= R_FSR_W;
            end
            R_FSR_W: if (rsp_valid) begin
                c_valid <= 1'b0;
                if (rsp_err) begin rx_err_code <= axi_err; state <= R_ERR; end
                else if (rsp_rdata[14:8] == 7'd0) state <= R_WAIT;      // FIFO empty
                else begin
                    ri <= rsp_rdata[5:0]; k <= 5'd0; rx_data <= 512'd0; state <= R_RD_I;
                end
            end

            // k = 0 ID, k = 1 DLC, k >= 2 data words
            R_RD_I: begin
                c_we    <= 1'b0;
                c_addr  <= rxb + roff;
                c_valid <= 1'b1;
                state   <= R_RD_W;
            end
            R_RD_W: if (rsp_valid) begin
                c_valid <= 1'b0;
                if (rsp_err) begin rx_err_code <= axi_err; state <= R_ERR; end
                else begin
                    if (k == 5'd0) rx_id <= rsp_rdata;
                    else if (k == 5'd1) begin
                        rx_dlc <= rsp_rdata; nw <= nw_now; rx_nwords <= nw_now;
                    end else
                        rx_data[{widx, 5'b00000} +: 32] <= rsp_rdata;

                    if (k != 5'd0 && k == nw_now + 5'd1) state <= R_IRI_I;
                    else begin k <= k + 1'b1; state <= R_RD_I; end
                end
            end

            // advance the FIFO read index (write IRI = 1)
            R_IRI_I: begin
                c_we <= 1'b1; c_addr <= base_l + {20'd0, ADDR_FSR};
                c_wdata <= 32'h0000_0080; c_valid <= 1'b1; state <= R_IRI_W;
            end
            R_IRI_W: if (rsp_valid) begin
                c_valid <= 1'b0;
                if (rsp_err) begin rx_err_code <= axi_err; state <= R_ERR; end
                else begin rx_valid <= 1'b1; state <= R_OUT; end
            end

            R_OUT: if (rx_ready) begin rx_valid <= 1'b0; state <= R_WAIT; end

            R_ERR: begin rx_error <= 1'b1; c_valid <= 1'b0; state <= R_WAIT; end
            default: state <= R_ERR;
            endcase
        end
    end
endmodule
