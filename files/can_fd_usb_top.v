// ---------------------------------------------------------------------
// can_fd_usb_top.v
// Multi-channel CAN FD <-> USB bridge (FT2232H) top level.
//
//   PC --(Ch B async)--> ftdi_chb_rx_fsm -> can_tx_router -> TX FIFO[i]
//                                         -> can_channel[i] -> AXI-Lite -> CAN FD IP[i]
//   PC <-(Ch A sync )--- ftdi_cha_tx_fsm <- can_rx_mux    <- RX FIFO[i]
//
// One AXI4-Lite master per channel, each with its own set of ports:
// m_axi_0_* ... m_axi_7_*.
// NUM_CH (1..8) selects how many channels are built; each has its own AXI4-Lite
// master and expects its own CAN FD IP. The ports of channels at or above NUM_CH
// are tied off (outputs 0, inputs ignored). How many of the built channels are
// used, every register value and AUTO_CFG come from the configuration frame.
// Channel i's CAN FD core address = BASE0 + i * BASE_STRIDE (4 KB aligned).
// ---------------------------------------------------------------------
module can_fd_usb_top #(
    parameter integer NUM_CH = 8,            // number of channels built, 1 .. 8
    parameter [31:0]  BASE0       = 32'h4000_0000,
    parameter [31:0]  BASE_STRIDE = 32'h0001_0000,

    parameter integer FIFO_AW = 12        // 4096 x 32 per FIFO
)(
    input  wire        aclk,              // system / AXI clock
    input  wire        aresetn,

    // configuration (one start per channel)
    input  wire [NUM_CH-1:0]   cfg_start,
    output wire [NUM_CH-1:0]   cfg_busy,
    output wire [NUM_CH-1:0]   cfg_done,
    output wire [NUM_CH-1:0]   cfg_error,
    output wire [3*NUM_CH-1:0] cfg_err_code,
    output wire [5*NUM_CH-1:0] cfg_err_idx,

    // FT2232H channel A (sync 245 FIFO, FPGA -> PC)
    input  wire        ft_clk,            // ACBUS5 CLKOUT, 60 MHz
    input  wire        fta_txe_n,
    output wire        fta_wr_n,
    output wire [7:0]  fta_data,
    output wire        fta_rd_n,          // tied inactive
    output wire        fta_oe_n,          // tied inactive

    // status
    output wire [16*NUM_CH-1:0] frames_sent,
    output wire [16*NUM_CH-1:0] frames_err,
    output wire [3*NUM_CH-1:0]  last_tx_err,
    output wire [NUM_CH-1:0]    rx_error,
    output wire [3*NUM_CH-1:0]  rx_err_code,
    output wire [NUM_CH-1:0]    tx_fifo_full,
    output wire [15:0]          router_drop_cnt,
    output wire [15:0]          early_drop_cnt,   // data frames dropped before the configuration frame started the channels
    output wire                 cfg_applied,      // configuration done: channels started and data accepted
    output wire                 cfg_incomplete,   // an active channel had no BRPR/BTR in the configuration frame
    output wire [3:0]           active_ch,        // channels enabled by the configuration frame

    // ---- AXI4-Lite master of channel 0 (present when NUM_CH > 0) ----
    output wire [31:0] m_axi_0_awaddr,
    output wire        m_axi_0_awvalid,
    output wire [31:0] m_axi_0_wdata,
    output wire [3:0] m_axi_0_wstrb,
    output wire        m_axi_0_wvalid,
    output wire        m_axi_0_bready,
    output wire [31:0] m_axi_0_araddr,
    output wire        m_axi_0_arvalid,
    output wire        m_axi_0_rready,
    input  wire        m_axi_0_awready,
    input  wire        m_axi_0_wready,
    input  wire [1:0] m_axi_0_bresp,
    input  wire        m_axi_0_bvalid,
    input  wire        m_axi_0_arready,
    input  wire [31:0] m_axi_0_rdata,
    input  wire [1:0] m_axi_0_rresp,
    input  wire        m_axi_0_rvalid,

    // ---- AXI4-Lite master of channel 1 (present when NUM_CH > 1) ----
    output wire [31:0] m_axi_1_awaddr,
    output wire        m_axi_1_awvalid,
    output wire [31:0] m_axi_1_wdata,
    output wire [3:0] m_axi_1_wstrb,
    output wire        m_axi_1_wvalid,
    output wire        m_axi_1_bready,
    output wire [31:0] m_axi_1_araddr,
    output wire        m_axi_1_arvalid,
    output wire        m_axi_1_rready,
    input  wire        m_axi_1_awready,
    input  wire        m_axi_1_wready,
    input  wire [1:0] m_axi_1_bresp,
    input  wire        m_axi_1_bvalid,
    input  wire        m_axi_1_arready,
    input  wire [31:0] m_axi_1_rdata,
    input  wire [1:0] m_axi_1_rresp,
    input  wire        m_axi_1_rvalid,

    // ---- AXI4-Lite master of channel 2 (present when NUM_CH > 2) ----
    output wire [31:0] m_axi_2_awaddr,
    output wire        m_axi_2_awvalid,
    output wire [31:0] m_axi_2_wdata,
    output wire [3:0] m_axi_2_wstrb,
    output wire        m_axi_2_wvalid,
    output wire        m_axi_2_bready,
    output wire [31:0] m_axi_2_araddr,
    output wire        m_axi_2_arvalid,
    output wire        m_axi_2_rready,
    input  wire        m_axi_2_awready,
    input  wire        m_axi_2_wready,
    input  wire [1:0] m_axi_2_bresp,
    input  wire        m_axi_2_bvalid,
    input  wire        m_axi_2_arready,
    input  wire [31:0] m_axi_2_rdata,
    input  wire [1:0] m_axi_2_rresp,
    input  wire        m_axi_2_rvalid,

    // ---- AXI4-Lite master of channel 3 (present when NUM_CH > 3) ----
    output wire [31:0] m_axi_3_awaddr,
    output wire        m_axi_3_awvalid,
    output wire [31:0] m_axi_3_wdata,
    output wire [3:0] m_axi_3_wstrb,
    output wire        m_axi_3_wvalid,
    output wire        m_axi_3_bready,
    output wire [31:0] m_axi_3_araddr,
    output wire        m_axi_3_arvalid,
    output wire        m_axi_3_rready,
    input  wire        m_axi_3_awready,
    input  wire        m_axi_3_wready,
    input  wire [1:0] m_axi_3_bresp,
    input  wire        m_axi_3_bvalid,
    input  wire        m_axi_3_arready,
    input  wire [31:0] m_axi_3_rdata,
    input  wire [1:0] m_axi_3_rresp,
    input  wire        m_axi_3_rvalid,

    // ---- AXI4-Lite master of channel 4 (present when NUM_CH > 4) ----
    output wire [31:0] m_axi_4_awaddr,
    output wire        m_axi_4_awvalid,
    output wire [31:0] m_axi_4_wdata,
    output wire [3:0] m_axi_4_wstrb,
    output wire        m_axi_4_wvalid,
    output wire        m_axi_4_bready,
    output wire [31:0] m_axi_4_araddr,
    output wire        m_axi_4_arvalid,
    output wire        m_axi_4_rready,
    input  wire        m_axi_4_awready,
    input  wire        m_axi_4_wready,
    input  wire [1:0] m_axi_4_bresp,
    input  wire        m_axi_4_bvalid,
    input  wire        m_axi_4_arready,
    input  wire [31:0] m_axi_4_rdata,
    input  wire [1:0] m_axi_4_rresp,
    input  wire        m_axi_4_rvalid,

    // ---- AXI4-Lite master of channel 5 (present when NUM_CH > 5) ----
    output wire [31:0] m_axi_5_awaddr,
    output wire        m_axi_5_awvalid,
    output wire [31:0] m_axi_5_wdata,
    output wire [3:0] m_axi_5_wstrb,
    output wire        m_axi_5_wvalid,
    output wire        m_axi_5_bready,
    output wire [31:0] m_axi_5_araddr,
    output wire        m_axi_5_arvalid,
    output wire        m_axi_5_rready,
    input  wire        m_axi_5_awready,
    input  wire        m_axi_5_wready,
    input  wire [1:0] m_axi_5_bresp,
    input  wire        m_axi_5_bvalid,
    input  wire        m_axi_5_arready,
    input  wire [31:0] m_axi_5_rdata,
    input  wire [1:0] m_axi_5_rresp,
    input  wire        m_axi_5_rvalid,

    // ---- AXI4-Lite master of channel 6 (present when NUM_CH > 6) ----
    output wire [31:0] m_axi_6_awaddr,
    output wire        m_axi_6_awvalid,
    output wire [31:0] m_axi_6_wdata,
    output wire [3:0] m_axi_6_wstrb,
    output wire        m_axi_6_wvalid,
    output wire        m_axi_6_bready,
    output wire [31:0] m_axi_6_araddr,
    output wire        m_axi_6_arvalid,
    output wire        m_axi_6_rready,
    input  wire        m_axi_6_awready,
    input  wire        m_axi_6_wready,
    input  wire [1:0] m_axi_6_bresp,
    input  wire        m_axi_6_bvalid,
    input  wire        m_axi_6_arready,
    input  wire [31:0] m_axi_6_rdata,
    input  wire [1:0] m_axi_6_rresp,
    input  wire        m_axi_6_rvalid,

    // ---- AXI4-Lite master of channel 7 (present when NUM_CH > 7) ----
    output wire [31:0] m_axi_7_awaddr,
    output wire        m_axi_7_awvalid,
    output wire [31:0] m_axi_7_wdata,
    output wire [3:0] m_axi_7_wstrb,
    output wire        m_axi_7_wvalid,
    output wire        m_axi_7_bready,
    output wire [31:0] m_axi_7_araddr,
    output wire        m_axi_7_arvalid,
    output wire        m_axi_7_rready,
    input  wire        m_axi_7_awready,
    input  wire        m_axi_7_wready,
    input  wire [1:0] m_axi_7_bresp,
    input  wire        m_axi_7_bvalid,
    input  wire        m_axi_7_arready,
    input  wire [31:0] m_axi_7_rdata,
    input  wire [1:0] m_axi_7_rresp,
    input  wire        m_axi_7_rvalid,

    // FT2232H channel B (async 245 FIFO, PC -> FPGA)
    input  wire [7:0]  ftb_data,
    input  wire        ftb_rxf_n,
    output wire        ftb_rd_n,
    output wire        ftb_wr_n          // tied inactive
);
    localparam integer MAX_CH = 8;

    // NUM_CH outside 1..MAX_CH names a module that does not exist: elaboration stops here
    generate
        if (NUM_CH < 1 || NUM_CH > MAX_CH) begin : g_bad_num_ch
            NUM_CH_must_be_between_1_and_8 bad_num_ch ();
        end
    endgenerate

    assign ftb_wr_n = 1'b1;               // channel B is receive-only

    // reset synchroniser for the ft_clk domain
    reg [1:0] ftr;
    always @(posedge ft_clk or negedge aresetn)
        if (!aresetn) ftr <= 2'b00; else ftr <= {ftr[0], 1'b1};
    wire ft_resetn = ftr[1];

    // ---------------- AXI port plumbing: per-channel ports <-> MAX_CH-wide vectors ----------------
    wire [32*MAX_CH-1:0] awaddr_v;
    wire [MAX_CH-1:0]    awvalid_v;
    wire [32*MAX_CH-1:0] wdata_v;
    wire [4*MAX_CH-1:0] wstrb_v;
    wire [MAX_CH-1:0]    wvalid_v;
    wire [MAX_CH-1:0]    bready_v;
    wire [32*MAX_CH-1:0] araddr_v;
    wire [MAX_CH-1:0]    arvalid_v;
    wire [MAX_CH-1:0]    rready_v;
    wire [MAX_CH-1:0]    awready_v;
    wire [MAX_CH-1:0]    wready_v;
    wire [2*MAX_CH-1:0] bresp_v;
    wire [MAX_CH-1:0]    bvalid_v;
    wire [MAX_CH-1:0]    arready_v;
    wire [32*MAX_CH-1:0] rdata_v;
    wire [2*MAX_CH-1:0] rresp_v;
    wire [MAX_CH-1:0]    rvalid_v;

    assign m_axi_0_awaddr = awaddr_v[32*0 +: 32];
    assign m_axi_0_awvalid = awvalid_v[0];
    assign m_axi_0_wdata = wdata_v[32*0 +: 32];
    assign m_axi_0_wstrb = wstrb_v[4*0 +: 4];
    assign m_axi_0_wvalid = wvalid_v[0];
    assign m_axi_0_bready = bready_v[0];
    assign m_axi_0_araddr = araddr_v[32*0 +: 32];
    assign m_axi_0_arvalid = arvalid_v[0];
    assign m_axi_0_rready = rready_v[0];
    assign m_axi_1_awaddr = awaddr_v[32*1 +: 32];
    assign m_axi_1_awvalid = awvalid_v[1];
    assign m_axi_1_wdata = wdata_v[32*1 +: 32];
    assign m_axi_1_wstrb = wstrb_v[4*1 +: 4];
    assign m_axi_1_wvalid = wvalid_v[1];
    assign m_axi_1_bready = bready_v[1];
    assign m_axi_1_araddr = araddr_v[32*1 +: 32];
    assign m_axi_1_arvalid = arvalid_v[1];
    assign m_axi_1_rready = rready_v[1];
    assign m_axi_2_awaddr = awaddr_v[32*2 +: 32];
    assign m_axi_2_awvalid = awvalid_v[2];
    assign m_axi_2_wdata = wdata_v[32*2 +: 32];
    assign m_axi_2_wstrb = wstrb_v[4*2 +: 4];
    assign m_axi_2_wvalid = wvalid_v[2];
    assign m_axi_2_bready = bready_v[2];
    assign m_axi_2_araddr = araddr_v[32*2 +: 32];
    assign m_axi_2_arvalid = arvalid_v[2];
    assign m_axi_2_rready = rready_v[2];
    assign m_axi_3_awaddr = awaddr_v[32*3 +: 32];
    assign m_axi_3_awvalid = awvalid_v[3];
    assign m_axi_3_wdata = wdata_v[32*3 +: 32];
    assign m_axi_3_wstrb = wstrb_v[4*3 +: 4];
    assign m_axi_3_wvalid = wvalid_v[3];
    assign m_axi_3_bready = bready_v[3];
    assign m_axi_3_araddr = araddr_v[32*3 +: 32];
    assign m_axi_3_arvalid = arvalid_v[3];
    assign m_axi_3_rready = rready_v[3];
    assign m_axi_4_awaddr = awaddr_v[32*4 +: 32];
    assign m_axi_4_awvalid = awvalid_v[4];
    assign m_axi_4_wdata = wdata_v[32*4 +: 32];
    assign m_axi_4_wstrb = wstrb_v[4*4 +: 4];
    assign m_axi_4_wvalid = wvalid_v[4];
    assign m_axi_4_bready = bready_v[4];
    assign m_axi_4_araddr = araddr_v[32*4 +: 32];
    assign m_axi_4_arvalid = arvalid_v[4];
    assign m_axi_4_rready = rready_v[4];
    assign m_axi_5_awaddr = awaddr_v[32*5 +: 32];
    assign m_axi_5_awvalid = awvalid_v[5];
    assign m_axi_5_wdata = wdata_v[32*5 +: 32];
    assign m_axi_5_wstrb = wstrb_v[4*5 +: 4];
    assign m_axi_5_wvalid = wvalid_v[5];
    assign m_axi_5_bready = bready_v[5];
    assign m_axi_5_araddr = araddr_v[32*5 +: 32];
    assign m_axi_5_arvalid = arvalid_v[5];
    assign m_axi_5_rready = rready_v[5];
    assign m_axi_6_awaddr = awaddr_v[32*6 +: 32];
    assign m_axi_6_awvalid = awvalid_v[6];
    assign m_axi_6_wdata = wdata_v[32*6 +: 32];
    assign m_axi_6_wstrb = wstrb_v[4*6 +: 4];
    assign m_axi_6_wvalid = wvalid_v[6];
    assign m_axi_6_bready = bready_v[6];
    assign m_axi_6_araddr = araddr_v[32*6 +: 32];
    assign m_axi_6_arvalid = arvalid_v[6];
    assign m_axi_6_rready = rready_v[6];
    assign m_axi_7_awaddr = awaddr_v[32*7 +: 32];
    assign m_axi_7_awvalid = awvalid_v[7];
    assign m_axi_7_wdata = wdata_v[32*7 +: 32];
    assign m_axi_7_wstrb = wstrb_v[4*7 +: 4];
    assign m_axi_7_wvalid = wvalid_v[7];
    assign m_axi_7_bready = bready_v[7];
    assign m_axi_7_araddr = araddr_v[32*7 +: 32];
    assign m_axi_7_arvalid = arvalid_v[7];
    assign m_axi_7_rready = rready_v[7];

    assign awready_v = {m_axi_7_awready, m_axi_6_awready, m_axi_5_awready, m_axi_4_awready, m_axi_3_awready, m_axi_2_awready, m_axi_1_awready, m_axi_0_awready};
    assign wready_v = {m_axi_7_wready, m_axi_6_wready, m_axi_5_wready, m_axi_4_wready, m_axi_3_wready, m_axi_2_wready, m_axi_1_wready, m_axi_0_wready};
    assign bresp_v = {m_axi_7_bresp, m_axi_6_bresp, m_axi_5_bresp, m_axi_4_bresp, m_axi_3_bresp, m_axi_2_bresp, m_axi_1_bresp, m_axi_0_bresp};
    assign bvalid_v = {m_axi_7_bvalid, m_axi_6_bvalid, m_axi_5_bvalid, m_axi_4_bvalid, m_axi_3_bvalid, m_axi_2_bvalid, m_axi_1_bvalid, m_axi_0_bvalid};
    assign arready_v = {m_axi_7_arready, m_axi_6_arready, m_axi_5_arready, m_axi_4_arready, m_axi_3_arready, m_axi_2_arready, m_axi_1_arready, m_axi_0_arready};
    assign rdata_v = {m_axi_7_rdata, m_axi_6_rdata, m_axi_5_rdata, m_axi_4_rdata, m_axi_3_rdata, m_axi_2_rdata, m_axi_1_rdata, m_axi_0_rdata};
    assign rresp_v = {m_axi_7_rresp, m_axi_6_rresp, m_axi_5_rresp, m_axi_4_rresp, m_axi_3_rresp, m_axi_2_rresp, m_axi_1_rresp, m_axi_0_rresp};
    assign rvalid_v = {m_axi_7_rvalid, m_axi_6_rvalid, m_axi_5_rvalid, m_axi_4_rvalid, m_axi_3_rvalid, m_axi_2_rvalid, m_axi_1_rvalid, m_axi_0_rvalid};

    // ---------------- run-time configuration (configuration frame from the PC) ----------------
    wire                    rt_wr, rt_commit;
    wire [15:0]             rt_id;
    wire [31:0]             rt_val;
    wire                    rt_configured, rt_incomplete;
    wire [3:0]              rt_active;
    wire [NUM_CH-1:0]       rt_start;
    wire [15:0]             rt_bad;
    wire [NUM_CH-1:0]       rt_rx_en;
    wire [5*NUM_CH-1:0]     rt_tx_buf;
    wire [NUM_CH*12*32-1:0] rt_vals;

    can_cfg_regs #(.NUM_CH(NUM_CH)) u_cfgregs (
        .aclk(aclk), .aresetn(aresetn),
        .wr_en(rt_wr), .wr_id(rt_id), .wr_val(rt_val), .commit(rt_commit),
        .active_ch(rt_active), .configured(rt_configured), .incomplete(rt_incomplete),
        .start(rt_start), .bad_cnt(rt_bad), .rx_en(rt_rx_en), .tx_buf(rt_tx_buf), .vals(rt_vals)
    );

    // nothing runs until the configuration frame has started the channels and is complete
    assign cfg_applied    = rt_configured;
    assign cfg_incomplete = rt_incomplete;
    assign active_ch      = rt_active;

    // ---------------- TX path: USB ch B -> router -> TX FIFOs ----------------
    wire [31:0]       usb_word;
    wire              usb_wr, usb_full;
    wire [31:0]       txf_din;
    wire [NUM_CH-1:0] txf_wr;

    ftdi_chb_rx_fsm u_chb (
        .aclk(aclk), .aresetn(aresetn),
        .ftb_data(ftb_data), .ftb_rxf_n(ftb_rxf_n), .ftb_rd_n(ftb_rd_n),
        .fifo_din(usb_word), .fifo_wr_en(usb_wr), .fifo_full(usb_full)
    );

    can_tx_router #(.NUM_CH(NUM_CH)) u_router (
        .aclk(aclk), .aresetn(aresetn),
        .in_data(usb_word), .in_wr(usb_wr), .in_full(usb_full),
        .fifo_din(txf_din), .fifo_wr(txf_wr), .fifo_full(tx_fifo_full),
        .active_ch(rt_active), .configured(rt_configured),
        .cfg_wr(rt_wr), .cfg_id(rt_id), .cfg_val(rt_val), .cfg_commit(rt_commit),
        .drop_cnt(router_drop_cnt), .early_cnt(early_drop_cnt)
    );

    // ---------------- RX path: RX FIFOs -> mux -> USB ch A ----------------
    wire [32*NUM_CH-1:0] rxf_dout;
    wire [NUM_CH-1:0]    rxf_empty, rxf_rd;
    wire [31:0]          mux_data;
    wire                 mux_empty, mux_rd;

    can_rx_mux #(.NUM_CH(NUM_CH)) u_mux (
        .clk(ft_clk), .rstn(ft_resetn),
        .fifo_dout(rxf_dout), .fifo_empty(rxf_empty), .fifo_rd(rxf_rd),
        .out_data(mux_data), .out_empty(mux_empty), .out_rd_en(mux_rd)
    );

    ftdi_cha_tx_fsm u_cha (
        .ft_clk(ft_clk), .ft_resetn(ft_resetn),
        .fta_txe_n(fta_txe_n), .fta_wr_n(fta_wr_n), .fta_data(fta_data),
        .fta_rd_n(fta_rd_n), .fta_oe_n(fta_oe_n),
        .fifo_dout(mux_data), .fifo_empty(mux_empty), .fifo_rd_en(mux_rd)
    );

    // ---------------- one channel per CAN FD IP ----------------
    genvar g;
    generate
        for (g = 0; g < NUM_CH; g = g + 1) begin : g_ch
            can_channel #(
                .BASE_ADDR(BASE0 + g * BASE_STRIDE),
                .FIFO_AW(FIFO_AW),
                .CH_ID(g)
            ) u_ch (
                .aclk(aclk), .aresetn(aresetn), .ft_clk(ft_clk), .ft_resetn(ft_resetn),

                .cfg_start(cfg_start[g] | rt_start[g]), .cfg_vals(rt_vals[g*12*32 +: 12*32]), .rx_en(rt_rx_en[g]), .tx_buf(rt_tx_buf[5*g +: 5]), .cfg_busy(cfg_busy[g]), .cfg_done(cfg_done[g]),
                .cfg_error(cfg_error[g]),
                .cfg_err_code(cfg_err_code[3*g +: 3]), .cfg_err_idx(cfg_err_idx[5*g +: 5]),

                .txf_din(txf_din), .txf_wr(txf_wr[g]), .txf_full(tx_fifo_full[g]),
                .rxf_dout(rxf_dout[32*g +: 32]), .rxf_empty(rxf_empty[g]), .rxf_rd(rxf_rd[g]),

                .frames_sent(frames_sent[16*g +: 16]), .frames_err(frames_err[16*g +: 16]),
                .last_tx_err(last_tx_err[3*g +: 3]),
                .rx_error(rx_error[g]), .rx_err_code(rx_err_code[3*g +: 3]),

                .m_axi_awaddr(awaddr_v[32*g +: 32]),
                .m_axi_awvalid(awvalid_v[g]),
                .m_axi_wdata(wdata_v[32*g +: 32]),
                .m_axi_wstrb(wstrb_v[4*g +: 4]),
                .m_axi_wvalid(wvalid_v[g]),
                .m_axi_bready(bready_v[g]),
                .m_axi_araddr(araddr_v[32*g +: 32]),
                .m_axi_arvalid(arvalid_v[g]),
                .m_axi_rready(rready_v[g]),
                .m_axi_awready(awready_v[g]),
                .m_axi_wready(wready_v[g]),
                .m_axi_bresp(bresp_v[2*g +: 2]),
                .m_axi_bvalid(bvalid_v[g]),
                .m_axi_arready(arready_v[g]),
                .m_axi_rdata(rdata_v[32*g +: 32]),
                .m_axi_rresp(rresp_v[2*g +: 2]),
                .m_axi_rvalid(rvalid_v[g])
            );
        end

        // channels above NUM_CH do not exist: their AXI outputs are held at 0
        for (g = NUM_CH; g < MAX_CH; g = g + 1) begin : g_unused
            assign awaddr_v[32*g +: 32] = 32'd0;
            assign awvalid_v[g] = 1'd0;
            assign wdata_v[32*g +: 32] = 32'd0;
            assign wstrb_v[4*g +: 4] = 4'd0;
            assign wvalid_v[g] = 1'd0;
            assign bready_v[g] = 1'd0;
            assign araddr_v[32*g +: 32] = 32'd0;
            assign arvalid_v[g] = 1'd0;
            assign rready_v[g] = 1'd0;
        end
    endgenerate
endmodule
