`timescale 1ns/1ps
// ---------------------------------------------------------------------
// tb_can_fd_usb.v  -  self-checking testbench (NOT yet run by the author:
// no simulator was available when this was written - run it first).
//
// Two behavioural CAN FD IP models sit behind the two AXI-Lite masters.
// A byte-level FT2232H model feeds channel B and captures channel A.
// Checks: auto-config, TX to ch1 / ch0, drop of an invalid channel,
//         RX from both channels through the mux.
// ---------------------------------------------------------------------

// ---------------- behavioural CAN FD IP (only what the FSMs touch) ----------------
module tb_can_ip_model (
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [31:0] awaddr,  input  wire awvalid,  output wire awready,
    input  wire [31:0] wdata,   input  wire wvalid,   output wire wready,
    output wire [1:0]  bresp,   output reg  bvalid,   input  wire bready,
    input  wire [31:0] araddr,  input  wire arvalid,  output wire arready,
    output reg  [31:0] rdata,   output wire [1:0] rresp, output reg rvalid, input wire rready
);
    reg [31:0] mem [0:4095];            // 16 KB register space
    reg        cen;
    reg [31:0] trr;
    reg [7:0]  trr_cnt;
    reg [6:0]  fl;                      // RX FIFO fill level
    reg [5:0]  ri;                      // RX FIFO read index
    reg [31:0] tx_count;
    reg        aw_got, w_got;
    reg [31:0] awa, wd;
    integer    i;

    assign awready = !aw_got && !bvalid;
    assign wready  = !w_got  && !bvalid;
    assign bresp   = 2'b00;
    assign arready = !rvalid;
    assign rresp   = 2'b00;

    initial begin
        for (i = 0; i < 4096; i = i + 1) mem[i] = 32'd0;
        fl = 0; ri = 0; tx_count = 0;
    end

    always @(posedge aclk) begin
        if (!aresetn) begin
            cen <= 0; trr <= 0; trr_cnt <= 0;
            aw_got <= 0; w_got <= 0; bvalid <= 0; rvalid <= 0; rdata <= 0;
        end else begin
            if (awvalid && awready) begin awa <= awaddr; aw_got <= 1; end
            if (wvalid  && wready ) begin wd  <= wdata;  w_got  <= 1; end

            if (aw_got && w_got && !bvalid) begin
                aw_got <= 0; w_got <= 0; bvalid <= 1;
                case (awa[13:2])
                    12'h000: begin                       // SRR
                        if (wd[0]) cen <= 0; else cen <= wd[1];
                    end
                    12'h024: if (wd != 0) begin trr <= wd; trr_cnt <= 8'd40; end   // TRR 0x090
                    12'h03A: if (wd[7] && fl != 0) begin fl <= fl - 1'b1; ri <= ri + 1'b1; end // FSR 0x0E8
                    default: mem[awa[13:2]] <= wd;
                endcase
            end
            if (bvalid && bready) bvalid <= 0;

            if (trr_cnt != 0) begin
                trr_cnt <= trr_cnt - 1'b1;
                if (trr_cnt == 8'd1) begin trr <= 0; tx_count <= tx_count + 1'b1; end
            end

            if (arvalid && arready) begin
                rvalid <= 1;
                case (araddr[13:2])
                    12'h006: rdata <= cen ? 32'h0000_0008 : 32'd0;       // SR 0x018
                    12'h024: rdata <= trr;                               // TRR
                    12'h03A: rdata <= {17'd0, fl, 2'b00, ri};            // FSR
                    default: rdata <= mem[araddr[13:2]];
                endcase
            end
            if (rvalid && rready) rvalid <= 0;
        end
    end
endmodule


// ---------------- testbench ----------------
module tb_can_fd_usb;
    localparam NUM_CH = 2;

    reg aclk = 0, ft_clk = 0, aresetn = 0;
    always #5     aclk   = ~aclk;      // 100 MHz
    always #8.333 ft_clk = ~ft_clk;    // 60 MHz

    // FT2232H models
    reg  [7:0] ub [0:511];
    integer    ub_n = 0, ub_i = 0;
    wire       ftb_rd_n;
    wire [7:0] ftb_data  = (ub_i < ub_n) ? ub[ub_i] : 8'h00;
    wire       ftb_rxf_n = !(ub_i < ub_n);
    always @(posedge ftb_rd_n) if (aresetn) #2 ub_i = ub_i + 1;

    wire       fta_wr_n, fta_rd_n, fta_oe_n, ftb_wr_n;
    wire [7:0] fta_data;
    reg  [7:0] cap [0:511];
    integer    cap_n = 0;
    always @(posedge ft_clk) if (fta_wr_n === 1'b0) begin cap[cap_n] = fta_data; cap_n = cap_n + 1; end

    // DUT
    reg  [NUM_CH-1:0] cfg_start = 0;
    wire [NUM_CH-1:0] cfg_busy, cfg_done, cfg_error, rx_error, tx_fifo_full;
    wire [3*NUM_CH-1:0] cfg_err_code, last_tx_err, rx_err_code;
    wire [5*NUM_CH-1:0] cfg_err_idx;
    wire [16*NUM_CH-1:0] frames_sent, frames_err;
    wire [15:0] router_drop_cnt;

    wire [32*NUM_CH-1:0] awaddr, wdata, araddr, rdata;
    wire [4*NUM_CH-1:0]  wstrb;
    wire [2*NUM_CH-1:0]  bresp, rresp;
    wire [NUM_CH-1:0]    awvalid, awready, wvalid, wready, bvalid, bready,
                         arvalid, arready, rvalid, rready;

    can_fd_usb_top #(.NUM_CH(NUM_CH), .FIFO_AW(6), .AUTO_CFG(1), .RX_EN(1)) dut (
        .aclk(aclk), .aresetn(aresetn),
        .cfg_start(cfg_start), .cfg_busy(cfg_busy), .cfg_done(cfg_done), .cfg_error(cfg_error),
        .cfg_err_code(cfg_err_code), .cfg_err_idx(cfg_err_idx),
        .ft_clk(ft_clk), .fta_txe_n(1'b0), .fta_wr_n(fta_wr_n), .fta_data(fta_data),
        .fta_rd_n(fta_rd_n), .fta_oe_n(fta_oe_n),
        .ftb_data(ftb_data), .ftb_rxf_n(ftb_rxf_n), .ftb_rd_n(ftb_rd_n), .ftb_wr_n(ftb_wr_n),
        .frames_sent(frames_sent), .frames_err(frames_err), .last_tx_err(last_tx_err),
        .rx_error(rx_error), .rx_err_code(rx_err_code), .tx_fifo_full(tx_fifo_full),
        .router_drop_cnt(router_drop_cnt),
        .m_axi_awaddr(awaddr), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
        .m_axi_araddr(araddr), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_rdata(rdata), .m_axi_rresp(rresp), .m_axi_rvalid(rvalid), .m_axi_rready(rready)
    );

    tb_can_ip_model u_ip0 (
        .aclk(aclk), .aresetn(aresetn),
        .awaddr(awaddr[31:0]), .awvalid(awvalid[0]), .awready(awready[0]),
        .wdata(wdata[31:0]),   .wvalid(wvalid[0]),   .wready(wready[0]),
        .bresp(bresp[1:0]),    .bvalid(bvalid[0]),   .bready(bready[0]),
        .araddr(araddr[31:0]), .arvalid(arvalid[0]), .arready(arready[0]),
        .rdata(rdata[31:0]),   .rresp(rresp[1:0]),   .rvalid(rvalid[0]), .rready(rready[0])
    );
    tb_can_ip_model u_ip1 (
        .aclk(aclk), .aresetn(aresetn),
        .awaddr(awaddr[63:32]), .awvalid(awvalid[1]), .awready(awready[1]),
        .wdata(wdata[63:32]),   .wvalid(wvalid[1]),   .wready(wready[1]),
        .bresp(bresp[3:2]),     .bvalid(bvalid[1]),   .bready(bready[1]),
        .araddr(araddr[63:32]), .arvalid(arvalid[1]), .arready(arready[1]),
        .rdata(rdata[63:32]),   .rresp(rresp[3:2]),   .rvalid(rvalid[1]), .rready(rready[1])
    );

    // helpers
    integer errs = 0;
    task push_word(input [31:0] w);
        begin
            ub[ub_n]   = w[7:0];   ub[ub_n+1] = w[15:8];
            ub[ub_n+2] = w[23:16]; ub[ub_n+3] = w[31:24];
            ub_n = ub_n + 4;
        end
    endtask
    function [31:0] getw(input integer i);
        getw = {cap[4*i+3], cap[4*i+2], cap[4*i+1], cap[4*i]};
    endfunction
    task expect32(input [255:0] name, input [31:0] got, input [31:0] exp);
        begin
            if (got !== exp) begin
                errs = errs + 1;
                $display("FAIL %0s: got %h expected %h", name, got, exp);
            end
        end
    endtask

    // one returned frame, at word index w: header must carry channel ch; every other field must equal the
    // identical frame that was injected on both channels
    task check_ident_frame(input [255:0] name, input integer w, input [7:0] ch);
        reg [31:0] hh;
        begin
            hh = getw(w);
            $display("%0s: header ch=%0d  ID=%h DLC=%h N=%0d D0=%h D1=%h",
                     name, hh[7:0], getw(w+1), getw(w+2), getw(w+3), getw(w+4), getw(w+5));
            expect32("ident channel tag", hh,         {24'd0, ch});
            expect32("ident ID",          getw(w+1),  32'h2460_0000);
            expect32("ident DLC",         getw(w+2),  32'h8000_0000);
            expect32("ident N",           getw(w+3),  32'd2);
            expect32("ident D0",          getw(w+4),  32'hA5A5_A5A5);
            expect32("ident D1",          getw(w+5),  32'h5A5A_5A5A);
        end
    endtask

    integer wi, t;
    reg [31:0] h;
    reg [1:0]  seen;

    // watchdog
    initial begin
        #3_000_000;
        $display("FAIL: timeout  cfg_done=%b sent=%h cap_n=%0d", cfg_done, frames_sent, cap_n);
        $finish;
    end

    initial begin
        repeat (20) @(posedge aclk);
        aresetn = 1;

        // ---- 1. auto configuration of both channels ----
        wait (cfg_done == 2'b11);
        $display("config done on both channels @ %0t", $time);
        expect32("cfg_error", {30'd0, cfg_error}, 32'd0);

        // ---- 2. TX: USB -> router -> channel FIFOs -> IP models ----
        push_word(32'd1);  push_word(32'h12345678); push_word(32'h4000_0000);
        push_word(32'd2);  push_word(32'hDEADBEEF); push_word(32'hCAFEBABE);   // ch1, 2 words
        push_word(32'd0);  push_word(32'h0000_00AB); push_word(32'h0000_0000);
        push_word(32'd0);                                                       // ch0, no data
        push_word(32'd5);  push_word(32'h0000_0001); push_word(32'h0000_0000);
        push_word(32'd1);  push_word(32'h0BAD_0BAD);                            // invalid ch 5

        t = 0;
        while (!(frames_sent[15:0] == 16'd1 && frames_sent[31:16] == 16'd1) && t < 200000) begin
            @(posedge aclk); t = t + 1;
        end
        expect32("ch0 frames_sent", {16'd0, frames_sent[15:0]},  32'd1);
        expect32("ch1 frames_sent", {16'd0, frames_sent[31:16]}, 32'd1);
        expect32("router_drop",     {16'd0, router_drop_cnt},    32'd1);
        expect32("ip1 tx ID",   u_ip1.mem[12'h040], 32'h12345678);
        expect32("ip1 tx DLC",  u_ip1.mem[12'h041], 32'h4000_0000);
        expect32("ip1 tx D0",   u_ip1.mem[12'h042], 32'hDEADBEEF);
        expect32("ip1 tx D1",   u_ip1.mem[12'h043], 32'hCAFEBABE);
        expect32("ip0 tx ID",   u_ip0.mem[12'h040], 32'h0000_00AB);
        expect32("ip0 tx_count", u_ip0.tx_count, 32'd1);
        expect32("ip1 tx_count", u_ip1.tx_count, 32'd1);

        // ---- 3. RX: IP models -> RX FSMs -> RX FIFOs -> mux -> USB ch A ----
        u_ip0.mem[12'h840] = 32'hAAAA0001;  u_ip0.mem[12'h841] = 32'h8000_0000;   // DLC 8 -> 2 words
        u_ip0.mem[12'h842] = 32'h11111111;  u_ip0.mem[12'h843] = 32'h22222222;
        u_ip1.mem[12'h840] = 32'hBBBB0002;  u_ip1.mem[12'h841] = 32'h2000_0000;   // DLC 2 -> 1 word
        u_ip1.mem[12'h842] = 32'h33333333;
        u_ip0.fl = 7'd1;
        u_ip1.fl = 7'd1;

        t = 0;
        while (cap_n < 44 && t < 200000) begin @(posedge ft_clk); t = t + 1; end
        expect32("USB ch A bytes", cap_n, 32'd44);

        wi = 0;
        repeat (2) begin
            h = getw(wi);
            if (h[7:0] == 8'd0) begin
                expect32("rx0 ID",  getw(wi+1), 32'hAAAA0001);
                expect32("rx0 DLC", getw(wi+2), 32'h8000_0000);
                expect32("rx0 N",   getw(wi+3), 32'd2);
                expect32("rx0 D0",  getw(wi+4), 32'h11111111);
                expect32("rx0 D1",  getw(wi+5), 32'h22222222);
                wi = wi + 6;
            end else if (h[7:0] == 8'd1) begin
                expect32("rx1 ID",  getw(wi+1), 32'hBBBB0002);
                expect32("rx1 DLC", getw(wi+2), 32'h2000_0000);
                expect32("rx1 N",   getw(wi+3), 32'd1);
                expect32("rx1 D0",  getw(wi+4), 32'h33333333);
                wi = wi + 5;
            end else begin
                errs = errs + 1; $display("FAIL: bad channel header %h", h); wi = wi + 1;
            end
        end

        // ---- 4. SAME CAN ID and DLC on both channels: only the channel header tells the frames apart ----
        // Both models popped one frame, so their RX read index is 1 -> buffer 1 at 0x2148 -> word index 0x852.
        u_ip0.mem[12'h852] = 32'h1234_5678;  u_ip0.mem[12'h853] = 32'h8000_0000;   // DLC 8 -> 2 words
        u_ip0.mem[12'h854] = 32'hC0DE_0000;  u_ip0.mem[12'h855] = 32'h0000_0A00;   // payload tagged "channel 0"
        u_ip1.mem[12'h852] = 32'h1234_5678;  u_ip1.mem[12'h853] = 32'h8000_0000;   // identical ID and DLC
        u_ip1.mem[12'h854] = 32'hC0DE_0001;  u_ip1.mem[12'h855] = 32'h0000_0B01;   // payload tagged "channel 1"
        u_ip0.fl = 7'd1;
        u_ip1.fl = 7'd1;

        t = 0;
        while (cap_n < 44 + 48 && t < 200000) begin @(posedge ft_clk); t = t + 1; end
        expect32("USB ch A bytes (2 more frames)", cap_n, 32'd92);

        seen = 2'b00;
        wi = 11;                                  // 44 bytes already consumed above
        repeat (2) begin
            h = getw(wi);
            $display("same-ID round: frame header ch=%0d  ID=%h DLC=%h N=%0d D0=%h D1=%h",
                     h[7:0], getw(wi+1), getw(wi+2), getw(wi+3), getw(wi+4), getw(wi+5));
            expect32("same-ID frame ID",  getw(wi+1), 32'h1234_5678);     // identical on both channels
            expect32("same-ID frame DLC", getw(wi+2), 32'h8000_0000);
            expect32("same-ID frame N",   getw(wi+3), 32'd2);
            if (h[7:0] == 8'd0) begin
                seen[0] = 1'b1;
                expect32("same-ID ch0 D0", getw(wi+4), 32'hC0DE_0000);
                expect32("same-ID ch0 D1", getw(wi+5), 32'h0000_0A00);
            end else if (h[7:0] == 8'd1) begin
                seen[1] = 1'b1;
                expect32("same-ID ch1 D0", getw(wi+4), 32'hC0DE_0001);
                expect32("same-ID ch1 D1", getw(wi+5), 32'h0000_0B01);
            end else begin
                errs = errs + 1; $display("FAIL: bad channel header %h", h);
            end
            wi = wi + 6;
        end
        expect32("both channels seen once", {30'd0, seen}, 32'd3);

        // ---- 5. IDENTICAL frames (same ID, DLC and payload) on both channels: the tag must follow the source ----
        // Each model's RX read index is now 2, so the next frame sits at 0x2190 (word 0x864); after it, at 0x21D8 (0x876).
        // 5a: channel 1 delivers first, channel 0 second.
        u_ip1.mem[12'h864] = 32'h2460_0000;  u_ip1.mem[12'h865] = 32'h8000_0000;
        u_ip1.mem[12'h866] = 32'hA5A5_A5A5;  u_ip1.mem[12'h867] = 32'h5A5A_5A5A;
        u_ip1.fl = 7'd1;
        t = 0;
        while (cap_n < 92 + 24 && t < 200000) begin @(posedge ft_clk); t = t + 1; end
        u_ip0.mem[12'h864] = 32'h2460_0000;  u_ip0.mem[12'h865] = 32'h8000_0000;
        u_ip0.mem[12'h866] = 32'hA5A5_A5A5;  u_ip0.mem[12'h867] = 32'h5A5A_5A5A;
        u_ip0.fl = 7'd1;
        t = 0;
        while (cap_n < 92 + 48 && t < 200000) begin @(posedge ft_clk); t = t + 1; end
        expect32("5a bytes on ch A", cap_n, 32'd140);
        check_ident_frame("5a 1st frame, injected on ch1", 23, 8'd1);
        check_ident_frame("5a 2nd frame, injected on ch0", 29, 8'd0);

        // 5b: the other way round, channel 0 first, channel 1 second.
        u_ip0.mem[12'h876] = 32'h2460_0000;  u_ip0.mem[12'h877] = 32'h8000_0000;
        u_ip0.mem[12'h878] = 32'hA5A5_A5A5;  u_ip0.mem[12'h879] = 32'h5A5A_5A5A;
        u_ip0.fl = 7'd1;
        t = 0;
        while (cap_n < 140 + 24 && t < 200000) begin @(posedge ft_clk); t = t + 1; end
        u_ip1.mem[12'h876] = 32'h2460_0000;  u_ip1.mem[12'h877] = 32'h8000_0000;
        u_ip1.mem[12'h878] = 32'hA5A5_A5A5;  u_ip1.mem[12'h879] = 32'h5A5A_5A5A;
        u_ip1.fl = 7'd1;
        t = 0;
        while (cap_n < 140 + 48 && t < 200000) begin @(posedge ft_clk); t = t + 1; end
        expect32("5b bytes on ch A", cap_n, 32'd188);
        check_ident_frame("5b 1st frame, injected on ch0", 35, 8'd0);
        check_ident_frame("5b 2nd frame, injected on ch1", 41, 8'd1);

        if (errs == 0) $display("PASS");
        else           $display("FAILED with %0d error(s)", errs);
        $finish;
    end
endmodule
