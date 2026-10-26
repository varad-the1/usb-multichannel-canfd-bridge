// ---------------------------------------------------------------------
// can_channel.v
// One complete channel = one CAN FD IP:
//   TX FIFO -> can_tx_fifo_fsm -+
//                               +-> can_ctrl_ch (cfg/TX/RX FSMs, arbiter,
//   RX FIFO <- can_rx_fifo_fsm -+                AXI-Lite master) -> IP
// The TX FIFO is written by the TX router (aclk domain).
// The RX FIFO is read by the RX mux (ft_clk domain), so it is also the
// aclk -> ft_clk clock-domain crossing.
// ---------------------------------------------------------------------
module can_channel #(
    parameter [31:0] BASE_ADDR = 32'h0000_0000,
    parameter [31:0] VAL_BRPR  = 32'h0000_0003,
    parameter [31:0] VAL_BTR   = 32'h0000_030E,
    parameter [31:0] VAL_FBRPR = 32'h0000_0000,
    parameter [31:0] VAL_FBTR  = 32'h0000_0000,
    parameter [31:0] VAL_MSR   = 32'h0000_0000,
    parameter [31:0] VAL_IER   = 32'h0000_0000,
    parameter [31:0] VAL_AFR   = 32'h0000_0003,
    parameter [31:0] SR_READY_MASK = 32'h0000_0008,
    parameter integer TX_BUF   = 0,
    parameter integer FIFO_AW  = 12,             // 2**12 = 4096 words
    parameter         AUTO_CFG = 0,              // 1 = run one config cycle after reset
    parameter integer RX_EN    = 1,              // 0 = TX only: the RX FSM never polls or pops
    parameter [7:0]   CH_ID    = 8'd0            // channel number, stored in front of every received frame
)(
    input  wire         aclk,
    input  wire         aresetn,
    input  wire         ft_clk,
    input  wire         ft_resetn,

    input  wire         cfg_start,
    output wire         cfg_busy,
    output wire         cfg_done,
    output wire         cfg_error,
    output wire [2:0]   cfg_err_code,
    output wire [4:0]   cfg_err_idx,

    // TX FIFO write side (aclk)
    input  wire [31:0]  txf_din,
    input  wire         txf_wr,
    output wire         txf_full,

    // RX FIFO read side (ft_clk)
    output wire [31:0]  rxf_dout,
    output wire         rxf_empty,
    input  wire         rxf_rd,

    // status
    output wire [15:0]  frames_sent,
    output wire [15:0]  frames_err,
    output wire [2:0]   last_tx_err,
    output wire         rx_error,
    output wire [2:0]   rx_err_code,

    // AXI4-Lite master to this channel's CAN FD IP
    output wire [31:0]  m_axi_awaddr,
    output wire         m_axi_awvalid,
    input  wire         m_axi_awready,
    output wire [31:0]  m_axi_wdata,
    output wire [3:0]   m_axi_wstrb,
    output wire         m_axi_wvalid,
    input  wire         m_axi_wready,
    input  wire [1:0]   m_axi_bresp,
    input  wire         m_axi_bvalid,
    output wire         m_axi_bready,
    output wire [31:0]  m_axi_araddr,
    output wire         m_axi_arvalid,
    input  wire         m_axi_arready,
    input  wire [31:0]  m_axi_rdata,
    input  wire [1:0]   m_axi_rresp,
    input  wire         m_axi_rvalid,
    output wire         m_axi_rready
);
    // ---------- optional power-up auto-configure ----------
    reg [11:0] auto_cnt;
    reg        auto_p;
    always @(posedge aclk) begin
        if (!aresetn) begin
            auto_cnt <= 12'd0; auto_p <= 1'b0;
        end else begin
            if (auto_cnt != 12'hFFF) auto_cnt <= auto_cnt + 1'b1;
            auto_p <= (auto_cnt >= 12'd2000) && (auto_cnt < 12'd2004);
        end
    end
    wire cfg_start_i = cfg_start | ((AUTO_CFG != 0) ? auto_p : 1'b0);

    // ---------- register-level controller ----------
    wire         tx_valid, tx_ready, tx_done, tx_error;
    wire [31:0]  tx_id, tx_dlc;
    wire [4:0]   tx_nwords;
    wire [511:0] tx_data;
    wire [2:0]   tx_err_code;
    wire         rx_valid, rx_ready;
    wire [31:0]  rx_id, rx_dlc;
    wire [4:0]   rx_nwords;
    wire [511:0] rx_data;

    can_ctrl_ch #(
        .BASE_ADDR(BASE_ADDR),
        .VAL_BRPR(VAL_BRPR), .VAL_BTR(VAL_BTR), .VAL_FBRPR(VAL_FBRPR), .VAL_FBTR(VAL_FBTR),
        .VAL_MSR(VAL_MSR), .VAL_IER(VAL_IER), .VAL_AFR(VAL_AFR),
        .SR_READY_MASK(SR_READY_MASK), .TX_BUF(TX_BUF), .RX_EN(RX_EN)
    ) u_ctrl (
        .aclk(aclk), .aresetn(aresetn),
        .cfg_start(cfg_start_i), .cfg_busy(cfg_busy), .cfg_done(cfg_done),
        .cfg_error(cfg_error), .cfg_err_code(cfg_err_code), .cfg_err_idx(cfg_err_idx),
        .tx_valid(tx_valid), .tx_ready(tx_ready), .tx_id(tx_id), .tx_dlc(tx_dlc),
        .tx_nwords(tx_nwords), .tx_data(tx_data),
        .tx_done(tx_done), .tx_error(tx_error), .tx_err_code(tx_err_code),
        .rx_valid(rx_valid), .rx_ready(rx_ready), .rx_id(rx_id), .rx_dlc(rx_dlc),
        .rx_nwords(rx_nwords), .rx_data(rx_data),
        .rx_error(rx_error), .rx_err_code(rx_err_code),
        .m_axi_awaddr(m_axi_awaddr), .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
        .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
        .m_axi_araddr(m_axi_araddr), .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready)
    );

    // ---------- TX side: TX FIFO -> frame FSM ----------
    wire [31:0] txf_dout;
    wire        txf_empty, txf_rd;

    async_fifo #(.W(32), .AW(FIFO_AW)) u_txfifo (
        .wclk(aclk), .wrstn(aresetn), .wr_en(txf_wr), .din(txf_din), .full(txf_full),
        .rclk(aclk), .rrstn(aresetn), .rd_en(txf_rd), .dout(txf_dout), .empty(txf_empty)
    );

    can_tx_fifo_fsm u_txf (
        .aclk(aclk), .aresetn(aresetn),
        .fifo_dout(txf_dout), .fifo_empty(txf_empty), .fifo_rd_en(txf_rd),
        .tx_valid(tx_valid), .tx_ready(tx_ready), .tx_id(tx_id), .tx_dlc(tx_dlc),
        .tx_nwords(tx_nwords), .tx_data(tx_data),
        .tx_done(tx_done), .tx_error(tx_error), .tx_err_code(tx_err_code),
        .frames_sent(frames_sent), .frames_err(frames_err), .last_err(last_tx_err)
    );

    // ---------- RX side: frame FSM -> RX FIFO (aclk -> ft_clk) ----------
    wire [31:0] rxf_din;
    wire        rxf_wr, rxf_full;

    can_rx_fifo_fsm #(.CH_ID(CH_ID)) u_rxf (
        .aclk(aclk), .aresetn(aresetn),
        .rx_valid(rx_valid), .rx_ready(rx_ready), .rx_id(rx_id), .rx_dlc(rx_dlc),
        .rx_nwords(rx_nwords), .rx_data(rx_data),
        .fifo_din(rxf_din), .fifo_wr_en(rxf_wr), .fifo_full(rxf_full)
    );

    async_fifo #(.W(32), .AW(FIFO_AW)) u_rxfifo (
        .wclk(aclk),   .wrstn(aresetn),   .wr_en(rxf_wr), .din(rxf_din), .full(rxf_full),
        .rclk(ft_clk), .rrstn(ft_resetn), .rd_en(rxf_rd), .dout(rxf_dout), .empty(rxf_empty)
    );
endmodule
