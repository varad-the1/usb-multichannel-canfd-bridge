// ---------------------------------------------------------------------
// can_fd_usb_top.v
// Multi-channel CAN FD <-> USB bridge (FT2232H) top level.
//
//   PC --(Ch B async)--> ftdi_chb_rx_fsm -> can_tx_router -> TX FIFO[i]
//                                         -> can_channel[i] -> AXI-Lite -> CAN FD IP[i]
//   PC <-(Ch A sync )--- ftdi_cha_tx_fsm <- can_rx_mux    <- RX FIFO[i]
//
// Every channel has its own FIFOs, TX/RX/config FSMs, arbiter and AXI-Lite
// master. The AXI ports are packed vectors: channel i uses slice i.
// Channel i's IP base address = BASE0 + i * BASE_STRIDE (4 KB aligned).
// ---------------------------------------------------------------------
module can_fd_usb_top #(
    parameter integer NUM_CH = 2,
    parameter [31:0]  BASE0       = 32'h4000_0000,
    parameter [31:0]  BASE_STRIDE = 32'h0001_0000,

    // per-channel values, channel i = bits [32*i +: 32]  (default: same for all)
    parameter [32*NUM_CH-1:0] VAL_BRPR_V  = {NUM_CH{32'h0000_0003}},
    parameter [32*NUM_CH-1:0] VAL_BTR_V   = {NUM_CH{32'h0000_030E}},
    parameter [32*NUM_CH-1:0] VAL_FBRPR_V = {NUM_CH{32'h0000_0000}},   // <-- FILL IN
    parameter [32*NUM_CH-1:0] VAL_FBTR_V  = {NUM_CH{32'h0000_0000}},   // <-- FILL IN
    parameter [32*NUM_CH-1:0] VAL_MSR_V   = {NUM_CH{32'h0000_0000}},
    parameter [32*NUM_CH-1:0] VAL_IER_V   = {NUM_CH{32'h0000_0000}},
    parameter [32*NUM_CH-1:0] VAL_AFR_V   = {NUM_CH{32'h0000_0003}},
    parameter [32*NUM_CH-1:0] SR_READY_V  = {NUM_CH{32'h0000_0008}},

    parameter integer TX_BUF  = 0,
    parameter integer FIFO_AW = 12,       // 4096 x 32 per FIFO
    parameter         AUTO_CFG = 0,
    parameter integer RX_EN    = 0       // 0 = TX only (default): never reads or pops the RX FIFO; 1 = also bridges RX to USB ch A
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

    // FT2232H channel B (async 245 FIFO, PC -> FPGA)
    input  wire [7:0]  ftb_data,
    input  wire        ftb_rxf_n,
    output wire        ftb_rd_n,
    output wire        ftb_wr_n,          // tied inactive

    // status
    output wire [16*NUM_CH-1:0] frames_sent,
    output wire [16*NUM_CH-1:0] frames_err,
    output wire [3*NUM_CH-1:0]  last_tx_err,
    output wire [NUM_CH-1:0]    rx_error,
    output wire [3*NUM_CH-1:0]  rx_err_code,
    output wire [NUM_CH-1:0]    tx_fifo_full,
    output wire [15:0]          router_drop_cnt,

    // AXI4-Lite masters, one per channel (packed)
    output wire [32*NUM_CH-1:0] m_axi_awaddr,
    output wire [NUM_CH-1:0]    m_axi_awvalid,
    input  wire [NUM_CH-1:0]    m_axi_awready,
    output wire [32*NUM_CH-1:0] m_axi_wdata,
    output wire [4*NUM_CH-1:0]  m_axi_wstrb,
    output wire [NUM_CH-1:0]    m_axi_wvalid,
    input  wire [NUM_CH-1:0]    m_axi_wready,
    input  wire [2*NUM_CH-1:0]  m_axi_bresp,
    input  wire [NUM_CH-1:0]    m_axi_bvalid,
    output wire [NUM_CH-1:0]    m_axi_bready,
    output wire [32*NUM_CH-1:0] m_axi_araddr,
    output wire [NUM_CH-1:0]    m_axi_arvalid,
    input  wire [NUM_CH-1:0]    m_axi_arready,
    input  wire [32*NUM_CH-1:0] m_axi_rdata,
    input  wire [2*NUM_CH-1:0]  m_axi_rresp,
    input  wire [NUM_CH-1:0]    m_axi_rvalid,
    output wire [NUM_CH-1:0]    m_axi_rready
);
    assign ftb_wr_n = 1'b1;               // channel B is receive-only

    // reset synchroniser for the ft_clk domain
    reg [1:0] ftr;
    always @(posedge ft_clk or negedge aresetn)
        if (!aresetn) ftr <= 2'b00; else ftr <= {ftr[0], 1'b1};
    wire ft_resetn = ftr[1];

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
        .drop_cnt(router_drop_cnt)
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
                .VAL_BRPR(VAL_BRPR_V   >> (32*g)),
                .VAL_BTR(VAL_BTR_V     >> (32*g)),
                .VAL_FBRPR(VAL_FBRPR_V >> (32*g)),
                .VAL_FBTR(VAL_FBTR_V   >> (32*g)),
                .VAL_MSR(VAL_MSR_V     >> (32*g)),
                .VAL_IER(VAL_IER_V     >> (32*g)),
                .VAL_AFR(VAL_AFR_V     >> (32*g)),
                .SR_READY_MASK(SR_READY_V >> (32*g)),
                .TX_BUF(TX_BUF), .FIFO_AW(FIFO_AW), .AUTO_CFG(AUTO_CFG), .RX_EN(RX_EN)
            ) u_ch (
                .aclk(aclk), .aresetn(aresetn), .ft_clk(ft_clk), .ft_resetn(ft_resetn),

                .cfg_start(cfg_start[g]), .cfg_busy(cfg_busy[g]), .cfg_done(cfg_done[g]),
                .cfg_error(cfg_error[g]),
                .cfg_err_code(cfg_err_code[3*g +: 3]), .cfg_err_idx(cfg_err_idx[5*g +: 5]),

                .txf_din(txf_din), .txf_wr(txf_wr[g]), .txf_full(tx_fifo_full[g]),
                .rxf_dout(rxf_dout[32*g +: 32]), .rxf_empty(rxf_empty[g]), .rxf_rd(rxf_rd[g]),

                .frames_sent(frames_sent[16*g +: 16]), .frames_err(frames_err[16*g +: 16]),
                .last_tx_err(last_tx_err[3*g +: 3]),
                .rx_error(rx_error[g]), .rx_err_code(rx_err_code[3*g +: 3]),

                .m_axi_awaddr(m_axi_awaddr[32*g +: 32]), .m_axi_awvalid(m_axi_awvalid[g]),
                .m_axi_awready(m_axi_awready[g]),
                .m_axi_wdata(m_axi_wdata[32*g +: 32]),   .m_axi_wstrb(m_axi_wstrb[4*g +: 4]),
                .m_axi_wvalid(m_axi_wvalid[g]),          .m_axi_wready(m_axi_wready[g]),
                .m_axi_bresp(m_axi_bresp[2*g +: 2]),     .m_axi_bvalid(m_axi_bvalid[g]),
                .m_axi_bready(m_axi_bready[g]),
                .m_axi_araddr(m_axi_araddr[32*g +: 32]), .m_axi_arvalid(m_axi_arvalid[g]),
                .m_axi_arready(m_axi_arready[g]),
                .m_axi_rdata(m_axi_rdata[32*g +: 32]),   .m_axi_rresp(m_axi_rresp[2*g +: 2]),
                .m_axi_rvalid(m_axi_rvalid[g]),          .m_axi_rready(m_axi_rready[g])
            );
        end
    endgenerate
endmodule
