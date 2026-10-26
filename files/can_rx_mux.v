// ---------------------------------------------------------------------
// can_rx_mux.v   (ft_clk domain)
// Collects frames from the per-channel RX FIFOs and presents one word
// stream (first-word-fall-through) to the FT2232H channel A FSM.
// Round-robin between channels, one whole frame at a time.
//
// Each FIFO already holds complete frames:  CH, ID, DLC, N, data...  (the channel number is
// written by can_rx_fifo_fsm). The mux only picks a channel and passes its frame through.
//
// Stream frame (32-bit words):
//   word 0 : channel number (bits [7:0])
//   word 1 : ID,  word 2 : DLC,  word 3 : N,  word 4.. : N data words
// ---------------------------------------------------------------------
module can_rx_mux #(
    parameter integer NUM_CH = 2
)(
    input  wire                 clk,
    input  wire                 rstn,

    // per-channel RX FIFOs (FWFT, read side already in this clock domain)
    input  wire [32*NUM_CH-1:0] fifo_dout,
    input  wire [NUM_CH-1:0]    fifo_empty,
    output wire [NUM_CH-1:0]    fifo_rd,

    // single FWFT stream out
    output wire [31:0]          out_data,
    output wire                 out_empty,
    input  wire                 out_rd_en
);
    localparam [1:0] M_IDLE = 2'd0, M_PASS = 2'd2;

    reg [1:0] st;
    reg [7:0] sel;       // channel being forwarded
    reg [7:0] rr;        // last channel served
    reg [4:0] k;         // word index inside the frame
    reg [4:0] nw;        // data word count (from word 3)

    // ---- round-robin pick: first non-empty channel after rr ----
    integer j, ci;
    reg       found;
    reg [7:0] pick;
    always @* begin
        found = 1'b0;
        pick  = 8'd0;
        for (j = 0; j < NUM_CH; j = j + 1) begin
            ci = rr + 1 + j;
            if (ci >= NUM_CH) ci = ci - NUM_CH;     // rr+1+j < 2*NUM_CH
            if (!found && !fifo_empty[ci]) begin
                found = 1'b1;
                pick  = ci;
            end
        end
    end

    // ---- output side ----
    wire [31:0] sel_dout  = fifo_dout[{sel, 5'b00000} +: 32];
    wire        sel_empty = fifo_empty[sel];

    assign out_data  = sel_dout;
    assign out_empty = (st == M_PASS) ? sel_empty : 1'b1;

    wire pop = (st == M_PASS) && out_rd_en && !sel_empty;

    genvar g;
    generate
        for (g = 0; g < NUM_CH; g = g + 1) begin : g_rd
            assign fifo_rd[g] = pop && (sel == g);
        end
    endgenerate

    // FIFO word order is  CH(k=0), ID(k=1), DLC(k=2), N(k=3), data(k>=4)
    always @(posedge clk) begin
        if (!rstn) begin
            st <= M_IDLE; sel <= 8'd0; rr <= NUM_CH - 1; k <= 5'd0; nw <= 5'd0;
        end else begin
            case (st)
            M_IDLE: if (found) begin
                sel <= pick; k <= 5'd0; st <= M_PASS;
            end

            M_PASS: if (pop) begin
                if (k == 5'd3) begin
                    nw <= sel_dout[4:0];
                    if (sel_dout[4:0] == 5'd0) begin
                        st <= M_IDLE; rr <= sel;
                    end else
                        k <= k + 1'b1;
                end else if (k > 5'd3) begin
                    if (k == nw + 5'd3) begin
                        st <= M_IDLE; rr <= sel;
                    end else
                        k <= k + 1'b1;
                end else
                    k <= k + 1'b1;
            end

            default: st <= M_IDLE;
            endcase
        end
    end
endmodule
