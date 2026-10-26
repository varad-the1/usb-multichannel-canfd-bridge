// ---------------------------------------------------------------------
// async_fifo.v
// Dual-clock FIFO, gray-code pointers, first-word-fall-through read.
//   - dout is valid whenever empty == 0
//   - asserting rd_en while empty == 0 pops the current word
// Default: 4096 x 32 (AW = 12). For a single-clock FIFO tie wclk = rclk.
// ---------------------------------------------------------------------
module async_fifo #(
    parameter W  = 32,
    parameter AW = 12
)(
    input  wire         wclk,
    input  wire         wrstn,
    input  wire         wr_en,
    input  wire [W-1:0] din,
    output wire         full,

    input  wire         rclk,
    input  wire         rrstn,
    input  wire         rd_en,
    output reg  [W-1:0] dout,
    output wire         empty
);
    reg [W-1:0] mem [0:(1<<AW)-1];

    reg [AW:0] wbin, wgray, rbin, rgray;
    reg [AW:0] rgray_w1, rgray_w2;   // read pointer synchronised into wclk
    reg [AW:0] wgray_r1, wgray_r2;   // write pointer synchronised into rclk

    // ---------------- write side ----------------
    assign full = (wgray == {~rgray_w2[AW:AW-1], rgray_w2[AW-2:0]});

    wire        wr_go   = wr_en & ~full;
    wire [AW:0] wbin_n  = wbin + {{AW{1'b0}}, wr_go};
    wire [AW:0] wgray_n = (wbin_n >> 1) ^ wbin_n;

    always @(posedge wclk)
        if (wr_go) mem[wbin[AW-1:0]] <= din;

    always @(posedge wclk) begin
        if (!wrstn) begin
            wbin <= 0; wgray <= 0; rgray_w1 <= 0; rgray_w2 <= 0;
        end else begin
            wbin     <= wbin_n;
            wgray    <= wgray_n;
            rgray_w1 <= rgray;
            rgray_w2 <= rgray_w1;
        end
    end

    // ---------------- read side ----------------
    assign empty = (rgray == wgray_r2);

    wire        rd_go   = rd_en & ~empty;
    wire [AW:0] rbin_n  = rbin + {{AW{1'b0}}, rd_go};
    wire [AW:0] rgray_n = (rbin_n >> 1) ^ rbin_n;

    // prefetch: dout always holds mem[rbin]
    always @(posedge rclk)
        dout <= mem[rbin_n[AW-1:0]];

    always @(posedge rclk) begin
        if (!rrstn) begin
            rbin <= 0; rgray <= 0; wgray_r1 <= 0; wgray_r2 <= 0;
        end else begin
            rbin     <= rbin_n;
            rgray    <= rgray_n;
            wgray_r1 <= wgray;
            wgray_r2 <= wgray_r1;
        end
    end
endmodule
