// ---------------------------------------------------------------------
// can_cfg_regs.v
// Run-time configuration registers of the bridge. There are NO built-in
// defaults: after reset every register is 0 and the bridge is idle. The first
// frame from the PC (the configuration frame, decoded by can_tx_router) decides
// everything: how many channels are active, every CAN FD register value, and
// whether the channels start by themselves (AUTO_CFG).
//
// Register id (16 bits) = { channel[7:0], index[7:0] }.  channel 0xFF = all channels.
//   index 0x00 : NUM_ACTIVE   number of enabled channels, clamped to 1..NUM_CH (channel field ignored)
//   index 0x01 : BRPR      0x02 : BTR      0x03 : FBRPR    0x04 : FBTR
//   index 0x05 : MSR       0x06 : IER      0x07 : AFR
//   index 0x08 : AFMR0     0x09 : AFIR0    0x0A : AFMR1    0x0B : AFIR1
//   index 0x0C : WMR
//   index 0x0D : AUTO_CFG  1 = the active channels start (run their CAN FD config sequence)
//                          as soon as this frame ends; 0 = values are only stored (global)
//   index 0x0E : START     non-zero = start the active channels at the end of this frame,
//                          whatever AUTO_CFG says (global)
//   index 0x0F : RX_EN     1 = the channel reads received frames from its CAN FD core and sends them
//                          to the PC; 0 = TX only (per channel, 0 after reset)
//   index 0x10 : TX_BUF    which CAN FD TX buffer (0..31) the channel transmits through (per channel)
// Anything else (unknown index, channel >= NUM_CH and not 0xFF) is ignored and counted in bad_cnt.
//
// At the end of a frame (commit) the channels are started only if AUTO_CFG is 1 or START was
// written in that frame. Every ACTIVE channel whose BRPR and BTR were both written gets a start
// pulse. If an active channel is missing BRPR or BTR, 'incomplete' goes high, nothing is started
// and 'configured' stays low. Channels at or above NUM_ACTIVE are never started.
// ---------------------------------------------------------------------
module can_cfg_regs #(
    parameter integer NUM_CH = 8              // channels built, 1 .. 8
)(
    input  wire                   aclk,
    input  wire                   aresetn,

    // from the router
    input  wire                   wr_en,
    input  wire [15:0]            wr_id,
    input  wire [31:0]            wr_val,
    input  wire                   commit,

    // to the rest of the bridge
    output reg  [3:0]             active_ch,     // number of enabled channels (0 until configured)
    output reg                    configured,    // channels started and configuration complete
    output reg                    incomplete,    // an active channel had no BRPR/BTR
    output reg  [NUM_CH-1:0]      start,         // per-channel cfg_start (held high for 8 cycles)
    output reg  [15:0]            bad_cnt,       // ignored register writes
    output reg  [NUM_CH-1:0]      rx_en,         // per channel RX enable
    output reg  [5*NUM_CH-1:0]    tx_buf,        // per channel TX buffer index
    output wire [NUM_CH*12*32-1:0] vals          // channel i, register k at [(i*12+k)*32 +: 32]
);
    localparam integer NREG = 12;

    reg [31:0] r [0:NUM_CH*NREG-1];
    reg [NUM_CH-1:0] got_brpr, got_btr;
    reg        auto_cfg;                       // register 0x0D
    reg        start_req;                      // register 0x0E written in this frame
    reg [3:0]  hold;
    reg [NUM_CH-1:0] go;                       // channels that can be started

    wire [7:0] chf = wr_id[15:8];
    wire [7:0] ri  = wr_id[7:0];

    genvar gi;
    generate
        for (gi = 0; gi < NUM_CH*NREG; gi = gi + 1) begin : g_vals
            assign vals[gi*32 +: 32] = r[gi];
        end
    endgenerate

    integer i;
    always @* begin
        for (i = 0; i < NUM_CH; i = i + 1)
            go[i] = (i < active_ch) && got_brpr[i] && got_btr[i];
    end

    wire [NUM_CH-1:0] act_mask = (1 << active_ch) - 1;
    wire              all_ok   = (active_ch != 4'd0) && ((go & act_mask) == act_mask);

    always @(posedge aclk) begin
        if (!aresetn) begin
            active_ch  <= 4'd0;
            configured <= 1'b0;
            incomplete <= 1'b0;
            start      <= {NUM_CH{1'b0}};
            bad_cnt    <= 16'd0;
            hold       <= 4'd0;
            got_brpr   <= {NUM_CH{1'b0}};
            got_btr    <= {NUM_CH{1'b0}};
            auto_cfg   <= 1'b0;
            start_req  <= 1'b0;
            rx_en      <= {NUM_CH{1'b0}};
            tx_buf     <= {5*NUM_CH{1'b0}};
            for (i = 0; i < NUM_CH*NREG; i = i + 1) r[i] <= 32'd0;
        end else begin
            if (wr_en) begin
                if (ri == 8'h00) begin
                    if (wr_val == 32'd0)            active_ch <= 4'd1;
                    else if (wr_val > NUM_CH)       active_ch <= NUM_CH;
                    else                            active_ch <= wr_val[3:0];
                end else if (ri >= 8'h01 && ri <= 8'h0C && (chf == 8'hFF || chf < NUM_CH)) begin
                    for (i = 0; i < NUM_CH; i = i + 1)
                        if (chf == 8'hFF || chf == i) begin
                            r[i*NREG + ri - 1] <= wr_val;
                            if (ri == 8'h01) got_brpr[i] <= 1'b1;
                            if (ri == 8'h02) got_btr[i]  <= 1'b1;
                        end
                end else if ((ri == 8'h0F || ri == 8'h10) && (chf == 8'hFF || chf < NUM_CH)) begin
                    for (i = 0; i < NUM_CH; i = i + 1)
                        if (chf == 8'hFF || chf == i) begin
                            if (ri == 8'h0F) rx_en[i]            <= wr_val[0];
                            else             tx_buf[5*i +: 5]    <= wr_val[4:0];
                        end
                end else if (ri == 8'h0D) begin
                    auto_cfg <= (wr_val != 32'd0);
                end else if (ri == 8'h0E) begin
                    if (wr_val != 32'd0) start_req <= 1'b1;
                end else
                    bad_cnt <= bad_cnt + 1'b1;
            end

            // end of the frame: start the channels if asked to, and only if every active channel has its bit timing
            if (commit) begin
                start_req <= 1'b0;
                if (auto_cfg || start_req) begin
                    incomplete <= !all_ok;
                    configured <= all_ok;
                    hold       <= 4'd8;
                end
            end else if (hold != 4'd0)
                hold <= hold - 1'b1;

            for (i = 0; i < NUM_CH; i = i + 1)
                start[i] <= (hold != 4'd0) && go[i] && all_ok;
        end
    end
endmodule
