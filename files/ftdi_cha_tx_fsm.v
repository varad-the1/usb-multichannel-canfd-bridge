// ---------------------------------------------------------------------
// ftdi_cha_tx_fsm.v
// FT2232H channel A, SYNCHRONOUS 245 FIFO, transmit only (FPGA -> PC).
// Runs on the 60 MHz ft_clk. Reads a first-word-fall-through word source
// and sends each 32-bit word as 4 bytes, least-significant byte first.
// OE# and RD# stay inactive so the FT2232H never drives the bus.
// ---------------------------------------------------------------------
module ftdi_cha_tx_fsm (
    input  wire        ft_clk,
    input  wire        ft_resetn,

    input  wire        fta_txe_n,
    output wire        fta_wr_n,
    output wire [7:0]  fta_data,
    output wire        fta_rd_n,
    output wire        fta_oe_n,

    input  wire [31:0] fifo_dout,
    input  wire        fifo_empty,
    output wire        fifo_rd_en
);
    assign fta_rd_n = 1'b1;
    assign fta_oe_n = 1'b1;

    reg [1:0] bidx;

    // the chip captures a byte on every clk edge where TXE# = 0 and WR# = 0,
    // so WR# is derived combinationally from TXE# and the source state
    wire adv = !fta_txe_n && !fifo_empty;

    assign fta_wr_n   = ~adv;
    assign fta_data   = fifo_dout[{bidx, 3'b000} +: 8];
    assign fifo_rd_en = adv && (bidx == 2'd3);

    always @(posedge ft_clk)
        if (!ft_resetn) bidx <= 2'd0;
        else if (adv)   bidx <= bidx + 1'b1;
endmodule
