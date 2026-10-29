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

    parameter [15:0] POLL_TIMEOUT  = 16'hFFFF       // number of SR reads
)(
    input  wire        aclk,
    input  wire        aresetn,

    input  wire        cfg_start,        // rising edge starts a cycle
    // register values, 32 bits each: 0 BRPR, 1 BTR, 2 FBRPR, 3 FBTR, 4 MSR, 5 IER, 6 AFR, 7 AFMR0, 8 AFIR0, 9 AFMR1, 10 AFIR1, 11 WMR
    input  wire [32*12-1:0] cfg_vals,
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

    wire [31:0] VAL_BRPR  = cfg_vals[32*0  +: 32];
    wire [31:0] VAL_BTR   = cfg_vals[32*1  +: 32];
    wire [31:0] VAL_FBRPR = cfg_vals[32*2  +: 32];
    wire [31:0] VAL_FBTR  = cfg_vals[32*3  +: 32];
    wire [31:0] VAL_MSR   = cfg_vals[32*4  +: 32];
    wire [31:0] VAL_IER   = cfg_vals[32*5  +: 32];
    wire [31:0] VAL_AFR   = cfg_vals[32*6  +: 32];
    wire [31:0] VAL_AFMR0 = cfg_vals[32*7  +: 32];
    wire [31:0] VAL_AFIR0 = cfg_vals[32*8  +: 32];
    wire [31:0] VAL_AFMR1 = cfg_vals[32*9  +: 32];
    wire [31:0] VAL_AFIR1 = cfg_vals[32*10 +: 32];
    wire [31:0] VAL_WMR   = cfg_vals[32*11 +: 32];

    // loopback (MSR bit 1) reports SR bit 1, normal mode reports SR bit 3
    wire [31:0] ready_mask = VAL_MSR[1] ? 32'h0000_0002 : 32'h0000_0008;

    reg [43:0] rom_cur;
    reg [4:0]  idx;
    always @* begin
        case (idx)
            5'd0:    rom_cur = {ADDR_SRR,   32'h0000_0001};  // SRST
            5'd1:    rom_cur = {ADDR_SRR,   32'h0000_0000};  // release, CEN = 0
            5'd2:    rom_cur = {ADDR_BRPR,  VAL_BRPR};
            5'd3:    rom_cur = {ADDR_BTR,   VAL_BTR};
            5'd4:    rom_cur = {ADDR_FBRPR, VAL_FBRPR};
            5'd5:    rom_cur = {ADDR_FBTR,  VAL_FBTR};
            5'd6:    rom_cur = {ADDR_MSR,   VAL_MSR};
            5'd7:    rom_cur = {ADDR_WMR,   VAL_WMR};        // RXFP (only writable while CEN = 0)
            5'd8:    rom_cur = {ADDR_AFR,   32'h0000_0000};  // filters off while editing
            5'd9:    rom_cur = {ADDR_AFMR0, VAL_AFMR0};
            5'd10:   rom_cur = {ADDR_AFIR0, VAL_AFIR0};
            5'd11:   rom_cur = {ADDR_AFMR1, VAL_AFMR1};
            5'd12:   rom_cur = {ADDR_AFIR1, VAL_AFIR1};
            5'd13:   rom_cur = {ADDR_AFR,   VAL_AFR};        // enable filter(s)
            5'd14:   rom_cur = {ADDR_IER,   VAL_IER};
            default: rom_cur = {ADDR_SRR,   32'h0000_0002};  // CEN = 1
        endcase
    end

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
    reg [15:0] delay_cnt;
    reg [15:0] poll_cnt;

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
                end else if ((rsp_rdata & ready_mask) == ready_mask) begin
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
