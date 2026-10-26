// ---------------------------------------------------------------------
// can_tx_router.v
// Receives the 32-bit word stream from the FT2232H channel B FSM and
// routes each frame to the TX FIFO of the addressed channel.
//
// USB frame (32-bit words, each sent little-endian):
//   word 0 : channel header, bits [7:0] = channel number (others reserved)
//   word 1 : ID register word
//   word 2 : DLC register word
//   word 3 : N = number of 32-bit data words (0..16)
//   word 4.. : N data words
//
// Forwarded to the channel FIFO: ID, DLC, N, data...  (header removed).
//  - channel >= NUM_CH      : whole frame dropped, drop_cnt incremented
//  - N > 16                 : N is clamped to 16, extra words discarded
//  - target FIFO full       : in_full goes high -> USB side stalls
// ---------------------------------------------------------------------
module can_tx_router #(
    parameter integer NUM_CH = 2
)(
    input  wire                aclk,
    input  wire                aresetn,

    // word stream in (from ftdi_chb_rx_fsm)
    input  wire [31:0]         in_data,
    input  wire                in_wr,
    output wire                in_full,

    // to the per-channel TX FIFOs
    output wire [31:0]         fifo_din,          // broadcast, qualified by fifo_wr
    output wire [NUM_CH-1:0]   fifo_wr,
    input  wire [NUM_CH-1:0]   fifo_full,

    output reg  [15:0]         drop_cnt
);
    localparam [2:0] R_HDR = 3'd0, R_ID = 3'd1, R_DLC = 3'd2, R_NW = 3'd3, R_DATA = 3'd4;

    reg [2:0] st;
    reg [7:0] ch_l;
    reg       ch_ok;
    reg [7:0] n_in;
    reg [7:0] cnt;

    wire tgt_full  = ch_ok ? fifo_full[ch_l] : 1'b0;
    assign in_full = (st == R_HDR) ? 1'b0 : tgt_full;

    // clamp N on its way into the FIFO
    wire [31:0] nw_fwd = (in_data[7:0] > 8'd16) ? 32'd16 : {24'd0, in_data[7:0]};
    assign fifo_din = (st == R_NW) ? nw_fwd : in_data;

    // forward in every state except the header; drop data words beyond 16
    wire fwd_en = in_wr && ch_ok && (st != R_HDR) && !((st == R_DATA) && (cnt >= 8'd16));

    genvar g;
    generate
        for (g = 0; g < NUM_CH; g = g + 1) begin : g_wr
            assign fifo_wr[g] = fwd_en && (ch_l == g);
        end
    endgenerate

    always @(posedge aclk) begin
        if (!aresetn) begin
            st <= R_HDR; ch_l <= 8'd0; ch_ok <= 1'b0; n_in <= 8'd0; cnt <= 8'd0;
            drop_cnt <= 16'd0;
        end else if (in_wr) begin
            case (st)
            R_HDR: begin
                ch_l  <= in_data[7:0];
                ch_ok <= (in_data[7:0] < NUM_CH);
                if (!(in_data[7:0] < NUM_CH)) drop_cnt <= drop_cnt + 1'b1;
                st    <= R_ID;
            end
            R_ID:  st <= R_DLC;
            R_DLC: st <= R_NW;
            R_NW: begin
                n_in <= in_data[7:0];
                cnt  <= 8'd0;
                st   <= (in_data[7:0] == 8'd0) ? R_HDR : R_DATA;
            end
            R_DATA: begin
                cnt <= cnt + 1'b1;
                if (cnt == n_in - 8'd1) st <= R_HDR;
            end
            default: st <= R_HDR;
            endcase
        end
    end
endmodule
