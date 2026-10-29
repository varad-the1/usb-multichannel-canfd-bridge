// ---------------------------------------------------------------------
// can_ctrl_ch.v
// Register-level controller for ONE CAN FD IP:
//   config FSM + TX FSM + RX FSM + per-channel arbiter + AXI-Lite master.
// Each channel owns its AXI master, so channels never contend for the bus.
// ---------------------------------------------------------------------
module can_ctrl_ch #(
    parameter [31:0] BASE_ADDR = 32'h0000_0000,   // 4 KB aligned
    parameter [15:0] RX_POLL_PERIOD = 16'd200,
    parameter [15:0] AXI_TIMEOUT    = 16'd1000
)(
    input  wire         aclk,
    input  wire         aresetn,

    // configuration
    input  wire         cfg_start,
    input  wire [32*12-1:0] cfg_vals,   // register values from can_cfg_regs
    input  wire         rx_en,          // RX path on (configuration frame)
    input  wire [4:0]   tx_buf,         // TX buffer to use (configuration frame)
    output wire         cfg_busy,
    output wire         cfg_done,
    output wire         cfg_error,
    output wire [2:0]   cfg_err_code,
    output wire [4:0]   cfg_err_idx,

    // TX user interface
    input  wire         tx_valid,
    output wire         tx_ready,
    input  wire [31:0]  tx_id,
    input  wire [31:0]  tx_dlc,
    input  wire [4:0]   tx_nwords,
    input  wire [511:0] tx_data,
    output wire         tx_done,
    output wire         tx_error,
    output wire [2:0]   tx_err_code,

    // RX user interface
    output wire         rx_valid,
    input  wire         rx_ready,
    output wire [31:0]  rx_id,
    output wire [31:0]  rx_dlc,
    output wire [4:0]   rx_nwords,
    output wire [511:0] rx_data,
    output wire         rx_error,
    output wire [2:0]   rx_err_code,

    // AXI4-Lite master (to this channel's CAN FD IP)
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
    // command buses from the three FSMs
    wire        cfg_cv, cfg_cwe, tx_cv, tx_cwe, rx_cv, rx_cwe;
    wire [31:0] cfg_ca, cfg_cd, tx_ca, tx_cd, rx_ca, rx_cd;
    wire        cfg_rv, tx_rv, rx_rv;

    // arbiter <-> master
    wire        m_cmd_valid, m_cmd_ready, m_cmd_we;
    wire [31:0] m_cmd_addr, m_cmd_wdata;
    wire        m_rsp_valid, m_rsp_err;
    wire [31:0] m_rsp_rdata;
    wire [1:0]  m_rsp_code;

    wire        quiesce, tx_idle, rx_idle;
    wire [31:0] active_base;
    wire        link_up = cfg_done & ~quiesce;
    assign cfg_busy = quiesce;

    can_cfg_fsm u_cfg (
        .aclk(aclk), .aresetn(aresetn),
        .cfg_start(cfg_start), .cfg_vals(cfg_vals), .base_addr(BASE_ADDR),
        .others_idle(tx_idle & rx_idle), .quiesce(quiesce),
        .cfg_done(cfg_done), .cfg_error(cfg_error),
        .err_code(cfg_err_code), .err_idx(cfg_err_idx), .active_base(active_base),
        .c_valid(cfg_cv), .c_we(cfg_cwe), .c_addr(cfg_ca), .c_wdata(cfg_cd),
        .rsp_valid(cfg_rv), .rsp_rdata(m_rsp_rdata), .rsp_err(m_rsp_err), .rsp_code(m_rsp_code)
    );

    can_tx_fsm u_tx (
        .aclk(aclk), .aresetn(aresetn), .link_up(link_up), .base_addr(active_base), .tx_buf(tx_buf),
        .tx_valid(tx_valid), .tx_ready(tx_ready), .tx_id(tx_id), .tx_dlc(tx_dlc),
        .tx_nwords(tx_nwords), .tx_data(tx_data),
        .tx_done(tx_done), .tx_error(tx_error), .tx_err_code(tx_err_code), .tx_idle(tx_idle),
        .c_valid(tx_cv), .c_we(tx_cwe), .c_addr(tx_ca), .c_wdata(tx_cd),
        .rsp_valid(tx_rv), .rsp_rdata(m_rsp_rdata), .rsp_err(m_rsp_err), .rsp_code(m_rsp_code)
    );

    can_rx_fsm #(.RX_POLL_PERIOD(RX_POLL_PERIOD)) u_rx (
        .aclk(aclk), .aresetn(aresetn), .link_up(link_up), .rx_en(rx_en), .base_addr(active_base),
        .rx_valid(rx_valid), .rx_ready(rx_ready), .rx_id(rx_id), .rx_dlc(rx_dlc),
        .rx_nwords(rx_nwords), .rx_data(rx_data),
        .rx_error(rx_error), .rx_err_code(rx_err_code), .rx_idle(rx_idle),
        .c_valid(rx_cv), .c_we(rx_cwe), .c_addr(rx_ca), .c_wdata(rx_cd),
        .rsp_valid(rx_rv), .rsp_rdata(m_rsp_rdata), .rsp_err(m_rsp_err), .rsp_code(m_rsp_code)
    );

    can_arbiter u_arb (
        .aclk(aclk), .aresetn(aresetn),
        .cfg_valid(cfg_cv), .cfg_we(cfg_cwe), .cfg_addr(cfg_ca), .cfg_wdata(cfg_cd), .cfg_rsp(cfg_rv),
        .tx_valid(tx_cv),   .tx_we(tx_cwe),   .tx_addr(tx_ca),   .tx_wdata(tx_cd),   .tx_rsp(tx_rv),
        .rx_valid(rx_cv),   .rx_we(rx_cwe),   .rx_addr(rx_ca),   .rx_wdata(rx_cd),   .rx_rsp(rx_rv),
        .cmd_valid(m_cmd_valid), .cmd_ready(m_cmd_ready), .cmd_we(m_cmd_we),
        .cmd_addr(m_cmd_addr), .cmd_wdata(m_cmd_wdata),
        .rsp_valid(m_rsp_valid)
    );

    axil_master #(.TIMEOUT(AXI_TIMEOUT)) u_axi (
        .aclk(aclk), .aresetn(aresetn),
        .cmd_valid(m_cmd_valid), .cmd_ready(m_cmd_ready),
        .cmd_we(m_cmd_we), .cmd_addr(m_cmd_addr), .cmd_wdata(m_cmd_wdata),
        .rsp_valid(m_rsp_valid), .rsp_rdata(m_rsp_rdata),
        .rsp_err(m_rsp_err), .rsp_code(m_rsp_code),
        .m_axi_awaddr(m_axi_awaddr), .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
        .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
        .m_axi_araddr(m_axi_araddr), .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready)
    );
endmodule
