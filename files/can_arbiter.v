// ---------------------------------------------------------------------
// can_arbiter.v
// Per-channel 3-port arbiter in front of one AXI-Lite master.
//   Priority : cfg > (TX / RX alternate)
//   One command is in flight at a time; the reply is routed back to the
//   requester that was granted.
// ---------------------------------------------------------------------
module can_arbiter (
    input  wire        aclk,
    input  wire        aresetn,

    // requester 0: config FSM
    input  wire        cfg_valid,
    input  wire        cfg_we,
    input  wire [31:0] cfg_addr,
    input  wire [31:0] cfg_wdata,
    output wire        cfg_rsp,

    // requester 1: TX FSM
    input  wire        tx_valid,
    input  wire        tx_we,
    input  wire [31:0] tx_addr,
    input  wire [31:0] tx_wdata,
    output wire        tx_rsp,

    // requester 2: RX FSM
    input  wire        rx_valid,
    input  wire        rx_we,
    input  wire [31:0] rx_addr,
    input  wire [31:0] rx_wdata,
    output wire        rx_rsp,

    // to AXI-Lite master
    output wire        cmd_valid,
    input  wire        cmd_ready,
    output wire        cmd_we,
    output wire [31:0] cmd_addr,
    output wire [31:0] cmd_wdata,
    input  wire        rsp_valid
);
    reg       busy;
    reg       last_tx;
    reg [2:0] gnt;                       // {rx, tx, cfg}

    wire any      = cfg_valid | tx_valid | rx_valid;
    wire pick_cfg = cfg_valid;
    wire pick_tx  = !cfg_valid && tx_valid && (!rx_valid || !last_tx);
    wire pick_rx  = !cfg_valid && rx_valid && !pick_tx;

    assign cmd_valid = !busy && any;
    assign cmd_we    = pick_cfg ? cfg_we    : pick_tx ? tx_we    : rx_we;
    assign cmd_addr  = pick_cfg ? cfg_addr  : pick_tx ? tx_addr  : rx_addr;
    assign cmd_wdata = pick_cfg ? cfg_wdata : pick_tx ? tx_wdata : rx_wdata;

    always @(posedge aclk) begin
        if (!aresetn) begin
            busy <= 1'b0; gnt <= 3'b000; last_tx <= 1'b0;
        end else if (!busy) begin
            if (cmd_valid && cmd_ready) begin
                busy <= 1'b1;
                gnt  <= {pick_rx, pick_tx, pick_cfg};
                if (pick_tx) last_tx <= 1'b1;
                if (pick_rx) last_tx <= 1'b0;
            end
        end else if (rsp_valid) begin
            busy <= 1'b0;
        end
    end

    assign cfg_rsp = rsp_valid & gnt[0];
    assign tx_rsp  = rsp_valid & gnt[1];
    assign rx_rsp  = rsp_valid & gnt[2];
endmodule
