// ---------------------------------------------------------------------
// ftdi_chb_rx_fsm.v
// FT2232H channel B, ASYNCHRONOUS 245 FIFO, receive only (PC -> FPGA).
// Reads bytes with the RXF#/RD# handshake, assembles little-endian 32-bit
// words and pushes them with a write strobe. Stops reading (USB stalls)
// while fifo_full is high.
// Defaults assume aclk = 100 MHz (RD# low >= 30 ns, RD# high >= 49 ns).
// ---------------------------------------------------------------------
module ftdi_chb_rx_fsm #(
    parameter RD_LOW_CYC  = 4,
    parameter RD_HIGH_CYC = 6
)(
    input  wire        aclk,
    input  wire        aresetn,

    input  wire [7:0]  ftb_data,       // input only; FPGA never drives channel B
    input  wire        ftb_rxf_n,
    output reg         ftb_rd_n,

    output reg  [31:0] fifo_din,
    output reg         fifo_wr_en,
    input  wire        fifo_full
);
    reg [1:0] rxf_sr;
    always @(posedge aclk) rxf_sr <= {rxf_sr[0], ftb_rxf_n};
    wire rxf_n_s = rxf_sr[1];

    localparam [1:0] S_IDLE = 2'd0, S_RD = 2'd1, S_GAP = 2'd2, S_PUSH = 2'd3;

    reg [1:0]  st;
    reg [3:0]  cnt;
    reg [1:0]  bidx;                  // byte index, wraps 3 -> 0
    reg [31:0] sh;

    always @(posedge aclk) begin
        if (!aresetn) begin
            st <= S_IDLE; cnt <= 4'd0; bidx <= 2'd0; sh <= 32'd0;
            ftb_rd_n <= 1'b1; fifo_wr_en <= 1'b0; fifo_din <= 32'd0;
        end else begin
            fifo_wr_en <= 1'b0;
            case (st)
            S_IDLE: if (!rxf_n_s) begin         // data waiting in the FT2232H
                ftb_rd_n <= 1'b0; cnt <= 4'd0; st <= S_RD;
            end

            S_RD: begin
                if (cnt == RD_LOW_CYC-1) begin
                    sh[{bidx, 3'b000} +: 8] <= ftb_data;   // sample just before RD# rises
                    ftb_rd_n <= 1'b1; cnt <= 4'd0; st <= S_GAP;
                end else cnt <= cnt + 1'b1;
            end

            S_GAP: begin                          // RD# high time + RXF# sync delay
                if (cnt == RD_HIGH_CYC-1) begin
                    bidx <= bidx + 1'b1;
                    st   <= (bidx == 2'd3) ? S_PUSH : S_IDLE;
                    cnt  <= 4'd0;
                end else cnt <= cnt + 1'b1;
            end

            S_PUSH: if (!fifo_full) begin         // back-pressure: USB stalls here
                fifo_din <= sh; fifo_wr_en <= 1'b1; st <= S_IDLE;
            end
            endcase
        end
    end
endmodule
