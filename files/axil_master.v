// ---------------------------------------------------------------------
// axil_master.v
// Minimal AXI4-Lite master: one command in, one response out.
//   cmd_valid/cmd_ready : command handshake (accepted when both high)
//   rsp_valid           : 1-cycle pulse when the transaction finishes
//   rsp_code            : 0 = OK, 1 = timeout, 2 = SLVERR/DECERR
// ---------------------------------------------------------------------
module axil_master #(
    parameter [15:0] TIMEOUT = 16'd1000       // aclk cycles per handshake/response
)(
    input  wire        aclk,
    input  wire        aresetn,

    input  wire        cmd_valid,
    output wire        cmd_ready,
    input  wire        cmd_we,
    input  wire [31:0] cmd_addr,
    input  wire [31:0] cmd_wdata,

    output reg         rsp_valid,
    output reg  [31:0] rsp_rdata,
    output reg         rsp_err,
    output reg  [1:0]  rsp_code,

    output reg  [31:0] m_axi_awaddr,
    output reg         m_axi_awvalid,
    input  wire        m_axi_awready,
    output reg  [31:0] m_axi_wdata,
    output wire [3:0]  m_axi_wstrb,
    output reg         m_axi_wvalid,
    input  wire        m_axi_wready,
    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output reg         m_axi_bready,
    output reg  [31:0] m_axi_araddr,
    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,
    input  wire [31:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rvalid,
    output reg         m_axi_rready
);
    assign m_axi_wstrb = 4'hF;

    localparam [2:0] M_IDLE = 3'd0,
                     M_WA   = 3'd1,   // write address + data phase
                     M_WB   = 3'd2,   // write response
                     M_RA   = 3'd3,   // read address
                     M_RD   = 3'd4;   // read data

    reg [2:0]  st;
    reg        aw_d, w_d;
    reg [15:0] tc;

    assign cmd_ready = (st == M_IDLE) && !rsp_valid;

    wire aw_hs = m_axi_awvalid && m_axi_awready;
    wire w_hs  = m_axi_wvalid  && m_axi_wready;
    wire ar_hs = m_axi_arvalid && m_axi_arready;

    always @(posedge aclk) begin
        if (!aresetn) begin
            st <= M_IDLE; aw_d <= 1'b0; w_d <= 1'b0; tc <= 16'd0;
            rsp_valid <= 1'b0; rsp_rdata <= 32'd0; rsp_err <= 1'b0; rsp_code <= 2'd0;
            m_axi_awaddr <= 32'd0; m_axi_awvalid <= 1'b0;
            m_axi_wdata  <= 32'd0; m_axi_wvalid  <= 1'b0; m_axi_bready <= 1'b0;
            m_axi_araddr <= 32'd0; m_axi_arvalid <= 1'b0; m_axi_rready <= 1'b0;
        end else begin
            rsp_valid <= 1'b0;

            case (st)
            M_IDLE: if (cmd_valid && cmd_ready) begin
                tc <= 16'd0;
                if (cmd_we) begin
                    m_axi_awaddr  <= cmd_addr;
                    m_axi_wdata   <= cmd_wdata;
                    m_axi_awvalid <= 1'b1;
                    m_axi_wvalid  <= 1'b1;
                    aw_d <= 1'b0; w_d <= 1'b0;
                    st   <= M_WA;
                end else begin
                    m_axi_araddr  <= cmd_addr;
                    m_axi_arvalid <= 1'b1;
                    st <= M_RA;
                end
            end

            M_WA: begin
                tc <= tc + 1'b1;
                if (aw_hs) begin m_axi_awvalid <= 1'b0; aw_d <= 1'b1; end
                if (w_hs)  begin m_axi_wvalid  <= 1'b0; w_d  <= 1'b1; end
                if ((aw_d || aw_hs) && (w_d || w_hs)) begin
                    m_axi_bready <= 1'b1;
                    tc <= 16'd0;
                    st <= M_WB;
                end else if (tc == TIMEOUT) begin
                    m_axi_awvalid <= 1'b0; m_axi_wvalid <= 1'b0;
                    rsp_valid <= 1'b1; rsp_err <= 1'b1; rsp_code <= 2'd1;
                    st <= M_IDLE;
                end
            end

            M_WB: begin
                tc <= tc + 1'b1;
                if (m_axi_bvalid && m_axi_bready) begin
                    m_axi_bready <= 1'b0;
                    rsp_valid <= 1'b1;
                    rsp_err   <= m_axi_bresp[1];
                    rsp_code  <= m_axi_bresp[1] ? 2'd2 : 2'd0;
                    st <= M_IDLE;
                end else if (tc == TIMEOUT) begin
                    m_axi_bready <= 1'b0;
                    rsp_valid <= 1'b1; rsp_err <= 1'b1; rsp_code <= 2'd1;
                    st <= M_IDLE;
                end
            end

            M_RA: begin
                tc <= tc + 1'b1;
                if (ar_hs) begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b1;
                    tc <= 16'd0;
                    st <= M_RD;
                end else if (tc == TIMEOUT) begin
                    m_axi_arvalid <= 1'b0;
                    rsp_valid <= 1'b1; rsp_err <= 1'b1; rsp_code <= 2'd1;
                    st <= M_IDLE;
                end
            end

            M_RD: begin
                tc <= tc + 1'b1;
                if (m_axi_rvalid && m_axi_rready) begin
                    m_axi_rready <= 1'b0;
                    rsp_rdata <= m_axi_rdata;
                    rsp_valid <= 1'b1;
                    rsp_err   <= m_axi_rresp[1];
                    rsp_code  <= m_axi_rresp[1] ? 2'd2 : 2'd0;
                    st <= M_IDLE;
                end else if (tc == TIMEOUT) begin
                    m_axi_rready <= 1'b0;
                    rsp_valid <= 1'b1; rsp_err <= 1'b1; rsp_code <= 2'd1;
                    st <= M_IDLE;
                end
            end

            default: st <= M_IDLE;
            endcase
        end
    end
endmodule
