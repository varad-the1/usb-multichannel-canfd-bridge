// ---------------------------------------------------------------------
// can_cfg_fsm.v
// Configuration FSM for ONE Xilinx CAN FD IP (PG223), AXI4-Lite via the
// per-channel arbiter. A rising edge on cfg_start runs one cycle:
//   wait TX/RX idle -> SRST -> release -> bit timing -> mode -> filters
//   -> IER -> CEN=1 -> poll SR until the requested mode is reported.
//
// !! Register offsets / bit positions are from memory of PG223.
// !! VERIFY every ADDR_* and the BTR/F_BTR packing for your IP version.
// ---------------------------------------------------------------------
module can_cfg_fsm #(
    parameter [11:0] ADDR_SRR   = 12'h000,
    parameter [11:0] ADDR_MSR   = 12'h004,
    parameter [11:0] ADDR_BRPR  = 12'h008,
    parameter [11:0] ADDR_BTR   = 12'h00C,
    parameter [11:0] ADDR_SR    = 12'h018,
    parameter [11:0] ADDR_IER   = 12'h020,
    parameter [11:0] ADDR_FBRPR = 12'h088,
    parameter [11:0] ADDR_FBTR  = 12'h08C,
    parameter [11:0] ADDR_AFR   = 12'h0E0,
    parameter [11:0] ADDR_AFMR0 = 12'hA00,
    parameter [11:0] ADDR_AFIR0 = 12'hA04,
    parameter [11:0] ADDR_AFMR1 = 12'hA08,
    parameter [11:0] ADDR_AFIR1 = 12'hA0C,
    parameter [11:0] ADDR_WMR   = 12'h0EC,         // RX FIFO watermark; RXFP = [20:16]

    parameter [31:0] VAL_BRPR   = 32'h0000_0003,
    parameter [31:0] VAL_BTR    = 32'h0000_030E,   // 1 Mbit/s with BRPR=3 on an 80 MHz can_clk (as in tb_dual_vip)
    parameter [31:0] VAL_FBRPR  = 32'h0000_0000,   // <-- FILL IN (incl. TDC)
    parameter [31:0] VAL_FBTR   = 32'h0000_0000,   // <-- FILL IN
    parameter [31:0] VAL_MSR    = 32'h0000_0000,   // 0 = normal
    parameter [31:0] VAL_IER    = 32'h0000_0000,
    // Two filters split the traffic between the RX FIFOs (PG223: with RXFP = 0, filter 0 -> RX FIFO-0,
    // filters above RXFP -> RX FIFO-1). Frames matching neither filter are not stored at all.
    parameter [31:0] VAL_AFR    = 32'h0000_0003,   // UAF0 and UAF1
    parameter [31:0] VAL_AFMR0  = 32'hFFFF_FFFF,   // filter 0: exact match, extended ID 0x18FECA00 -> RX FIFO-0
    parameter [31:0] VAL_AFIR0  = 32'hC7FD_9400,   //   {ID[28:18], SRR=1, IDE=1, ID[17:0], RTR=0}
    parameter [31:0] VAL_AFMR1  = 32'hFFF8_0000,   // filter 1: exact match, standard ID 0x123 -> RX FIFO-1
    parameter [31:0] VAL_AFIR1  = 32'h2460_0000,   //   {ID[28:18], SRR=0, IDE=0, 0, RTR=0}
    parameter [31:0] VAL_WMR    = 32'h0000_0F0F,   // RXFP = 0; both FIFO watermarks left at their reset value 0xF
    parameter [31:0] SR_READY_MASK = 32'h0000_0008, // NORMAL (bit3) / LBACK (bit1)
    parameter [15:0] POLL_TIMEOUT  = 16'hFFFF       // number of SR reads
)(
    input  wire        aclk,
    input  wire        aresetn,

    input  wire        cfg_start,        // rising edge starts a cycle
    input  wire [31:0] base_addr,        // 4 KB aligned
    input  wire        others_idle,      // TX FSM and RX FSM both idle

    output wire        quiesce,          // high while a cycle is pending/running
    output reg         cfg_done,         // high after a successful cycle
    output reg         cfg_error,
    output reg  [2:0]  err_code,         // 1 AXI timeout, 2 AXI resp, 3 SR poll timeout, 4 bad base
    output reg  [4:0]  err_idx,          // ROM entry being written when the error hit
    output reg  [31:0] active_base,

    // command port to the arbiter
    output reg         c_valid,
    output reg         c_we,
    output reg  [31:0] c_addr,
    output reg  [31:0] c_wdata,
    input  wire        rsp_valid,
    input  wire [31:0] rsp_rdata,
    input  wire        rsp_err,
    input  wire [1:0]  rsp_code
);
    localparam [2:0] ERR_NONE = 3'd0, ERR_AXI_TO = 3'd1, ERR_AXI_RESP = 3'd2,
                     ERR_POLL_TO = 3'd3, ERR_BASE = 3'd4;

    // ---- start edge detect ----
    reg [2:0] start_sr;
    always @(posedge aclk)
        if (!aresetn) start_sr <= 3'b000;
        else          start_sr <= {start_sr[1:0], cfg_start};
    wire start_rise = start_sr[1] & ~start_sr[2];

    // ---- write table {offset, data} ----
    localparam integer N_WR = 16;

    function [43:0] rom;
        input [4:0] i;
        begin
            case (i)
                5'd0:    rom = {ADDR_SRR,   32'h0000_0001};  // SRST
                5'd1:    rom = {ADDR_SRR,   32'h0000_0000};  // release, CEN = 0
                5'd2:    rom = {ADDR_BRPR,  VAL_BRPR};
                5'd3:    rom = {ADDR_BTR,   VAL_BTR};
                5'd4:    rom = {ADDR_FBRPR, VAL_FBRPR};
                5'd5:    rom = {ADDR_FBTR,  VAL_FBTR};
                5'd6:    rom = {ADDR_MSR,   VAL_MSR};
                5'd7:    rom = {ADDR_WMR,   VAL_WMR};        // RXFP (only writable while CEN = 0)
                5'd8:    rom = {ADDR_AFR,   32'h0000_0000};  // filters off while editing
                5'd9:    rom = {ADDR_AFMR0, VAL_AFMR0};
                5'd10:   rom = {ADDR_AFIR0, VAL_AFIR0};
                5'd11:   rom = {ADDR_AFMR1, VAL_AFMR1};
                5'd12:   rom = {ADDR_AFIR1, VAL_AFIR1};
                5'd13:   rom = {ADDR_AFR,   VAL_AFR};        // enable filter(s)
                5'd14:   rom = {ADDR_IER,   VAL_IER};
                default: rom = {ADDR_SRR,   32'h0000_0002};  // CEN = 1
            endcase
        end
    endfunction

    // ---- FSM ----
    localparam [3:0] S_IDLE  = 4'd0,
                     S_QUIET = 4'd1,
                     S_PRE   = 4'd2,
                     S_LOAD  = 4'd3,
                     S_WR_W  = 4'd4,
                     S_GAP   = 4'd5,
                     S_RD_I  = 4'd6,
                     S_RD_W  = 4'd7,
                     S_DONE  = 4'd8,
                     S_ERR   = 4'd9;

    localparam [15:0] PRE_DELAY = 16'd63;
    localparam [15:0] GAP_DELAY = 16'd63;

    reg [3:0]  state;
    reg [4:0]  idx;
    reg [15:0] delay_cnt;
    reg [15:0] poll_cnt;

    wire [43:0] rom_cur = rom(idx);
    wire [2:0]  axi_err = (rsp_code == 2'd1) ? ERR_AXI_TO : ERR_AXI_RESP;

    assign quiesce = (state != S_IDLE);

    always @(posedge aclk) begin
        if (!aresetn) begin
            state <= S_IDLE; idx <= 5'd0; delay_cnt <= 16'd0; poll_cnt <= 16'd0;
            cfg_done <= 1'b0; cfg_error <= 1'b0; err_code <= ERR_NONE; err_idx <= 5'd0;
            active_base <= 32'd0;
            c_valid <= 1'b0; c_we <= 1'b0; c_addr <= 32'd0; c_wdata <= 32'd0;
        end else begin
            case (state)

            S_IDLE: if (start_rise) begin
                active_base <= base_addr;
                cfg_done    <= 1'b0;
                cfg_error   <= 1'b0;
                err_code    <= ERR_NONE;
                err_idx     <= 5'd0;
                idx         <= 5'd0;
                poll_cnt    <= 16'd0;
                delay_cnt   <= 16'd0;
                if (base_addr[11:0] != 12'd0) begin
                    err_code <= ERR_BASE;
                    state    <= S_ERR;
                end else
                    state    <= S_QUIET;
            end

            // wait until this channel's TX and RX FSMs are idle
            S_QUIET: if (others_idle) state <= S_PRE;

            S_PRE: begin
                if (delay_cnt == PRE_DELAY) begin delay_cnt <= 16'd0; state <= S_LOAD; end
                else delay_cnt <= delay_cnt + 1'b1;
            end

            S_LOAD: begin
                c_we    <= 1'b1;
                c_addr  <= active_base + {20'd0, rom_cur[43:32]};
                c_wdata <= rom_cur[31:0];
                c_valid <= 1'b1;
                state   <= S_WR_W;
            end

            S_WR_W: if (rsp_valid) begin
                c_valid <= 1'b0;
                if (rsp_err) begin
                    err_code <= axi_err; err_idx <= idx; state <= S_ERR;
                end else begin
                    delay_cnt <= 16'd0; state <= S_GAP;
                end
            end

            S_GAP: begin
                if (delay_cnt == GAP_DELAY) begin
                    delay_cnt <= 16'd0;
                    if (idx == N_WR-1) begin
                        poll_cnt <= 16'd0; state <= S_RD_I;
                    end else begin
                        idx <= idx + 1'b1; state <= S_LOAD;
                    end
                end else
                    delay_cnt <= delay_cnt + 1'b1;
            end

            S_RD_I: begin
                c_we    <= 1'b0;
                c_addr  <= active_base + {20'd0, ADDR_SR};
                c_valid <= 1'b1;
                state   <= S_RD_W;
            end

            S_RD_W: if (rsp_valid) begin
                c_valid <= 1'b0;
                if (rsp_err) begin
                    err_code <= axi_err; err_idx <= idx; state <= S_ERR;
                end else if ((rsp_rdata & SR_READY_MASK) == SR_READY_MASK) begin
                    state <= S_DONE;
                end else if (poll_cnt == POLL_TIMEOUT) begin
                    err_code <= ERR_POLL_TO; err_idx <= idx; state <= S_ERR;
                end else begin
                    poll_cnt <= poll_cnt + 1'b1; state <= S_RD_I;
                end
            end

            S_DONE: begin
                cfg_done <= 1'b1;
                state    <= S_IDLE;
            end

            S_ERR: begin
                cfg_error <= 1'b1;
                cfg_done  <= 1'b0;
                c_valid   <= 1'b0;
                state     <= S_IDLE;
            end

            default: state <= S_ERR;
            endcase
        end
    end
endmodule
