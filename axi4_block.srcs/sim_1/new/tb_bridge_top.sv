`timescale 1ns / 1ps

// Two-way hybrid bench, EXTERNAL loopback. Both masters transmit AND receive on the one CAN FD core:
//
//   bridge (custom RTL, RX_EN = 1): TX buffer 0, RX FIFO-0, PC over the FT2232H (ch B in, ch A out)
//   axi_vip_0 (READ_WRITE)        : TX buffer 1, RX FIFO-1, direct AXI-Lite
//
// Acceptance filters (set by the bridge's config FSM, RXFP = 0) route by ID:
//   standard 0x123          -> filter 1 -> RX FIFO-1 -> the VIP
//   extended 0x18FECA00     -> filter 0 -> RX FIFO-0 -> the bridge -> PC
//
// Frame A: PC -> bridge -> core -> bus -> core -> RX FIFO-1 -> VIP          (standard 0x123)
// Frame B: VIP -> core -> bus -> core -> RX FIFO-0 -> bridge -> PC          (extended 0x18FECA00)
// Frame C: frame B read back by the PC and written to the bridge again, as standard 0x123
//          (bridge read -> bridge write): PC -> bridge -> core -> bus -> core -> RX FIFO-1 -> VIP
//
// A second-node model on the CAN pins ACKs each frame and replays it (a lone core in normal mode gets
// no ACK and never stores its own frames, PG223 Normal Mode). One frame is in flight at a time.
// Classic CAN, 8 data bytes 11 22 33 44 55 66 77 88 in both frames.
//
// Bridge cell settings assumed: RX_EN = 1, VAL_MSR_V = 0, SR_READY_V = 8, AUTO_CFG = 1.
// axi_vip_0 must be set to READ_WRITE.

module tb_bridge_top;

    import axi_vip_pkg::*;
    import axi_block_axi_vip_0_0_pkg::*;

    // Auxiliary test: 1 = also forward frame B back through the PC as frame C; 0 = only the normal A and B flows.
    localparam bit FORWARD_TEST = 1;

    localparam bit [31:0] CAN_BASE = 32'h44A0_0000;
    localparam bit [31:0] SR       = CAN_BASE + 32'h018;
    localparam bit [31:0] ESR      = CAN_BASE + 32'h014;   // PG223: ACKER = bit 4
    localparam bit [31:0] TRR      = CAN_BASE + 32'h090;
    localparam bit [31:0] FSR      = CAN_BASE + 32'h0E8;   // FIFO-0: FL [14:8], IRI [7], RI [5:0]; FIFO-1: FL_1 [30:24], IRI_1 [23], RI_1 [21:16]
    localparam bit [31:0] TB1_ID   = CAN_BASE + 32'h148;   // TX buffer 1 (stride 0x48), TRR bit 1
    localparam bit [31:0] TB1_DLC  = CAN_BASE + 32'h14C;
    localparam bit [31:0] TB1_DW0  = CAN_BASE + 32'h150;
    localparam bit [31:0] TB1_DW1  = CAN_BASE + 32'h154;
    localparam bit [31:0] RB1_BASE = CAN_BASE + 32'h4100;  // RX FIFO-1 buffer 0 (ID), stride 0x48
    localparam bit [31:0] RX_STRIDE = 32'h48;

    // ID / DLC words (PG223 TB*/RB*): standard 0x123 -> 0x24600000; extended 0x18FECA00 -> 0xC7FD9400
    localparam bit [31:0] ID_A  = 32'h2460_0000;
    localparam bit [31:0] ID_B  = 32'hC7FD_9400;
    localparam bit [31:0] DLC_W = 32'h8000_0000;            // DLC = 8, classic CAN
    localparam bit [31:0] D0    = 32'h1122_3344;
    localparam bit [31:0] D1    = 32'h5566_7788;

    // ---------------------------------------------------------
    // Clocks / reset
    // ---------------------------------------------------------
    bit clk = 0, ft_clk = 0, resetn = 0;
    always #5ns     clk    = ~clk;                   // 100 MHz system clock
    always #8.333ns ft_clk = ~ft_clk;                // 60 MHz FT2232H clock

    initial begin
        #10000ns;
        resetn = 1;
    end

    // ---------------------------------------------------------
    // FT2232H model: channel B (PC -> FPGA) at byte level, channel A (FPGA -> PC) captured
    // ---------------------------------------------------------
    reg  [7:0] ub [0:511];
    integer    ub_n = 0, ub_i = 0;
    wire       ftb_rd_n;
    wire [7:0] ftb_data  = (ub_i < ub_n) ? ub[ub_i] : 8'h00;
    wire       ftb_rxf_n = !(ub_i < ub_n);
    always @(posedge ftb_rd_n) if (resetn) #2 ub_i = ub_i + 1;

    wire       fta_wr_n, fta_rd_n, fta_oe_n, ftb_wr_n;
    wire [7:0] fta_data;
    reg  [7:0] cap [0:511];
    integer    cap_n = 0;
    always @(posedge ft_clk) if (fta_wr_n === 1'b0) begin cap[cap_n] = fta_data; cap_n = cap_n + 1; end

    // ---------------------------------------------------------
    // CAN bus: the core's TX ANDed with the second node (dominant 0 wins)
    // ---------------------------------------------------------
    wire can_tx;
    bit  resp_line = 1;                              // second node's output, 1 = recessive
    wire can_bus = can_tx & resp_line;

    wire [15:0] frames_sent;
    wire [31:0] gpio_out;

    axi_block_wrapper DUT (
        .CAN_INTERFACE_0_rx (can_bus),
        .CAN_INTERFACE_0_tx (can_tx),
        .clk                (clk),
        .reset_rtl_0        (resetn),
        .frames_sent_0      (frames_sent),
        .ft_clk_0           (ft_clk),
        .fta_data_0         (fta_data),
        .fta_oe_n_0         (fta_oe_n),
        .fta_rd_n_0         (fta_rd_n),
        .fta_txe_n_0        (1'b0),
        .fta_wr_n_0         (fta_wr_n),
        .ftb_data_0         (ftb_data),
        .ftb_rd_n_0         (ftb_rd_n),
        .ftb_rxf_n_0        (ftb_rxf_n),
        .ftb_wr_n_0         (ftb_wr_n),
        .gpio_rtl_0_tri_o   (gpio_out)
    );

    // ---------------------------------------------------------
    // Read/write AXI agent (axi_vip_0)
    // ---------------------------------------------------------
    axi_block_axi_vip_0_0_mst_t vip;

    initial begin
        vip = new("vip agent", DUT.axi_block_i.axi_vip_0.inst.IF);
        vip.set_agent_tag("RW VIP");
        vip.set_verbosity(0);
        vip.start_master();
    end

    int errors = 0;

    task automatic wr32(input bit [31:0] addr, input bit [31:0] data);
        xil_axi_resp_t resp;
        vip.AXI4LITE_WRITE_BURST(addr, 0, data, resp);
        if (resp !== XIL_AXI_RESP_OKAY) begin
            errors++;
            $error("[%0t] WR addr=0x%08h resp expected=OKAY actual=%0d", $time, addr, resp);
        end
    endtask

    task automatic rd32(input bit [31:0] addr, output bit [31:0] data);
        xil_axi_resp_t resp;
        bit [31:0] d;
        vip.AXI4LITE_READ_BURST(addr, 0, d, resp);
        if (resp !== XIL_AXI_RESP_OKAY) begin
            errors++;
            $error("[%0t] RD addr=0x%08h resp expected=OKAY actual=%0d", $time, addr, resp);
        end
        data = d;
    endtask

    task automatic chk(input string name, input bit [31:0] addr, input bit [31:0] exp, input bit [31:0] act);
        if (act !== exp) begin
            errors++;
            $error("[%0t] MISMATCH %s addr=0x%08h expected=0x%08h actual=0x%08h", $time, name, addr, exp, act);
        end
    endtask

    // ---------------------------------------------------------
    // Second CAN node. Listens to the core's frame, drives the ACK slot dominant, then replays
    // the same bits as a frame from "another node" (the core drives the ACK slot of the replay).
    // Handles standard/extended IDs, classic CAN and CAN FD with BRS = 0. A run of five equal
    // bits exactly at the end of the data field is not handled in CAN FD frames.
    // ---------------------------------------------------------
    localparam time BT = 1000ns;             // nominal bit time (BRPR = 3, BTR = 0x30E on 80 MHz can_clk)

    function automatic int fd_bytes(input int dlc);
        case (dlc)
            9:  return 12;  10: return 16;  11: return 20;  12: return 24;
            13: return 32;  14: return 48;  15: return 64;
            default: return dlc;
        endcase
    endfunction

    task automatic wait_until(input time t);
        if (t > $time) #(t - $time);
    endtask

    task automatic ext_responder();
        bit raw [$];
        time t0;
        int  i, n, run, ack_idx, bytes, dlc, f_pos, dlc_lo, data_end, crc_end, crc_raw, p;
        bit  b, prev, ide, fdf, brs, bad;
        forever begin
            wait (can_bus === 1'b1);
            @(negedge can_bus);                 // SOF
            if (can_bus !== 1'b0) continue;
            t0 = $time;
            raw.delete();
            i = 0; n = 0; run = 0; ack_idx = -1; dlc = 0; bytes = 0; bad = 0; prev = 0;
            ide = 0; fdf = 0; brs = 0; f_pos = 14; dlc_lo = 1000; data_end = -1; crc_end = -1;

            // --- decode while sampling at mid-bit. n counts destuffed bits (SOF = 0) ---
            //   standard: SOF, ID(11), RTR/RRS, IDE, FDF/r0 ...      extended: ... IDE, ID(18), RTR/RRS, FDF/r1 ...
            while (!bad && (ack_idx < 0 || i < ack_idx)) begin
                wait_until(t0 + i*BT + BT/2);
                b = can_bus;
                raw.push_back(b);
                if (ack_idx < 0) begin
                    if (run == 5) begin                 // dynamic stuff bit
                        run = 1; prev = b;
                    end else begin
                        run = (n == 0 || b != prev) ? 1 : run + 1;
                        prev = b;
                        if (n == 13) begin ide = b; f_pos = b ? 33 : 14; end   // IDE, then where FDF sits
                        else if (n == f_pos) fdf = b;
                        if (n > f_pos) begin
                            // CAN FD: res, BRS, ESI, DLC(4).  CAN: r0 (and r1 if extended), DLC(4)
                            dlc_lo = fdf ? f_pos + 4 : f_pos + (ide ? 2 : 1);
                            if (n >= dlc_lo && n < dlc_lo + 4) dlc = (dlc << 1) | b;
                            if (fdf && n == f_pos + 2) brs = b;
                        end
                        n++;
                        if (n == dlc_lo + 4 && data_end < 0) begin     // header done
                            bytes    = fdf ? fd_bytes(dlc) : ((dlc > 8) ? 8 : dlc);
                            data_end = n + 8*bytes;
                            if (brs) begin
                                bad = 1;
                                $display("[%0t] BUS: BRS=1 not supported, no ACK", $time);
                            end else
                                $display("[%0t] BUS: frame from the core, SOF at %0t, %s %s, DLC=%0d (%0d bytes)", $time, t0,
                                         ide ? "extended" : "standard", fdf ? "CAN FD" : "CAN", dlc, bytes);
                        end
                        if (!bad && n == data_end && ack_idx < 0 && crc_end < 0) begin
                            if (fdf) begin
                                // stuff count (4) + CRC (17, or 21 if > 16 bytes), fixed stuff bit before every 4
                                p       = (bytes <= 16) ? 21 : 25;
                                crc_raw = p + (p + 3) / 4;
                                ack_idx = i + crc_raw + 2;             // + CRC delimiter, then ACK slot
                            end else
                                crc_end = data_end + 15;               // classic: 15-bit CRC, still dynamically stuffed
                        end
                        if (!bad && crc_end >= 0 && n == crc_end && ack_idx < 0)
                            ack_idx = i + 2 + ((run == 5) ? 1 : 0);    // optional stuff bit, CRC delimiter, then ACK slot
                        if (ack_idx >= 0)
                            $display("[%0t] BUS: ACK slot = bit %0d", $time, ack_idx);
                    end
                end
                i++;
            end
            if (bad) begin
                wait_until($time + 20*BT);
                continue;
            end

            // --- 1) ACK the core's own frame ---
            wait_until(t0 + ack_idx*BT);
            resp_line = 1'b0;
            #(BT);
            resp_line = 1'b1;
            $display("[%0t] BUS: ACK driven", $time);

            // --- 2) wait out ACK delimiter + EOF(7) + IFS(3), then replay as another node ---
            wait_until(t0 + (ack_idx + 12 + 3)*BT);
            for (int j = 0; j < ack_idx; j++) begin
                resp_line = raw[j];
                #(BT);
            end
            resp_line = 1'b1;                   // ACK slot: the core drives it
            $display("[%0t] BUS: replayed frame, ACK slot left to the core", $time);
            wait_until($time + 9*BT);           // ACK slot, delimiter, EOF; the core's IFS follows
        end
    endtask

    initial begin
        wait (resetn === 1'b1);
        ext_responder();
    end

    // ---------------------------------------------------------
    // PC side: push a frame over USB channel B / read it back from channel A
    // ---------------------------------------------------------
    task automatic push_word(input bit [31:0] w);
        ub[ub_n]   = w[7:0];    ub[ub_n+1] = w[15:8];
        ub[ub_n+2] = w[23:16];  ub[ub_n+3] = w[31:24];
        ub_n = ub_n + 4;
    endtask

    function automatic bit [31:0] getw(input int i);
        return {cap[4*i+3], cap[4*i+2], cap[4*i+1], cap[4*i]};
    endfunction

    // ---------------------------------------------------------
    // Test
    // ---------------------------------------------------------
    initial begin
        bit [31:0] d, rxb, id, dlc, dw0, dw1, fwd_d0, fwd_d1;
        int tries;

        wait (resetn === 1'b1);
        #1000ns;

        // ===== Frame A: PC -> bridge TX -> bus -> RX FIFO-1 -> VIP =====
        push_word(32'd0);  push_word(ID_A);  push_word(DLC_W);  push_word(32'd2);
        push_word(D0);     push_word(D1);
        $display("[%0t] PC: frame A sent over USB (standard 0x123, ID word 0x%08h)", $time, ID_A);

        tries = 0;
        do begin
            #1000ns;
            rd32(FSR, d);
            tries++;
        end while (d[30:24] == 0 && tries < 1500);

        if (d[30:24] == 0) begin
            errors++;
            $error("[%0t] frame A: RX FIFO-1 stayed empty after %0d polls: addr=0x%08h actual=0x%08h", $time, tries, FSR, d);
            rd32(SR,  d);  $display("DIAG SR  = 0x%08h  NORMAL=%0b BBSY=%0b ESTAT=%0b%0b", d, d[3], d[5], d[8], d[7]);
            rd32(ESR, d);  $display("DIAG ESR = 0x%08h  ACKER=%0b BERR=%0b STER=%0b FMER=%0b CRCER=%0b", d, d[4], d[3], d[2], d[1], d[0]);
        end
        else begin
            rxb = RB1_BASE + RX_STRIDE * d[21:16];                  // oldest message: RX FIFO-1 read index RI_1
            rd32(rxb + 32'h0, id);  rd32(rxb + 32'h4, dlc);
            rd32(rxb + 32'h8, dw0); rd32(rxb + 32'hC, dw1);
            $display("[%0t] VIP: frame A in RX FIFO-1: FSR=0x%08h  ID=0x%08h DLC=0x%08h DW0=0x%08h DW1=0x%08h",
                     $time, d, id, dlc, dw0, dw1);
            chk("frame A ID",  rxb + 32'h0, ID_A,  id);
            chk("frame A DLC", rxb + 32'h4, DLC_W, dlc & 32'hFFE0_0000);   // [20:16] = matched filter, [15:0] = RX timestamp
            chk("frame A matched filter (1 = standard 0x123 -> RX FIFO-1)", rxb + 32'h4, 32'd1, {27'd0, dlc[20:16]});
            chk("frame A D0",  rxb + 32'h8, D0,    dw0);
            chk("frame A D1",  rxb + 32'hC, D1,    dw1);

            wr32(FSR, 32'h0080_0000);                               // IRI_1: pop RX FIFO-1
            #500ns;
            rd32(FSR, d);
            chk("RX FIFO-1 fill after pop", FSR, 32'd0, {25'd0, d[30:24]});
        end
        chk("frame A must not reach USB channel A", 32'h0, 32'd0, cap_n);

        // ===== Frame B: VIP -> TX buffer 1 -> bus -> RX FIFO-0 -> bridge -> PC =====
        wr32(TB1_ID,  ID_B);
        wr32(TB1_DLC, DLC_W);
        wr32(TB1_DW0, D0);
        wr32(TB1_DW1, D1);
        wr32(TRR, 32'h0000_0002);                                   // request TX buffer 1
        $display("[%0t] VIP: frame B requested (extended 0x18FECA00, ID word 0x%08h)", $time, ID_B);

        tries = 0;
        do begin
            #1000ns;
            rd32(TRR, d);
            tries++;
        end while (d[1] && tries < 1500);
        if (d[1]) begin
            errors++;
            $error("[%0t] frame B: TX buffer 1 never completed (TRR=0x%08h)", $time, d);
        end
        else $display("[%0t] VIP: TX buffer 1 completed (TRR cleared)", $time);

        tries = 0;
        while (cap_n < 24 && tries < 3000) begin #1000ns; tries++; end    // 6 words from the bridge
        #1000ns;
        $display("[%0t] PC: frame B back on channel A (%0d bytes): ch=%0d ID=0x%08h DLC=0x%08h N=%0d D0=0x%08h D1=0x%08h",
                 $time, cap_n, getw(0), getw(1), getw(2), getw(3), getw(4), getw(5));
        chk("frame B channel", 32'h0, 32'd0,  getw(0));
        chk("frame B ID",      32'h0, ID_B,   getw(1));
        chk("frame B DLC",     32'h0, DLC_W,  getw(2) & 32'hFFE0_0000);
        chk("frame B matched filter (0 = extended 0x18FECA00 -> RX FIFO-0)", 32'h0, 32'd0, (getw(2) >> 16) & 32'h1F);
        chk("frame B N",       32'h0, 32'd2,  getw(3));
        chk("frame B D0",      32'h0, D0,     getw(4));
        chk("frame B D1",      32'h0, D1,     getw(5));
        chk("bytes on USB channel A", 32'h0, 32'd24, cap_n);

        // ===== Frame C (forwarding through the PC): the PC reads frame B off channel A and writes it back =====
        //   bridge RX (read) -> USB ch A -> PC -> USB ch B -> bridge TX (write) -> bus -> RX FIFO-1 -> VIP
        // The DLC word read back carries the RX timestamp [15:0] and the matched filter index [20:16]; those are
        // reserved in a TX buffer, so only DLC/EDL/BRS are kept. The ID is changed to standard 0x123: forwarding
        // with the same ID would match filter 0 again and loop forever.
        if (FORWARD_TEST) begin
        fwd_d0 = getw(4);
        fwd_d1 = getw(5);
        push_word(getw(0));                          // channel
        push_word(ID_A);                             // new ID
        push_word(getw(2) & 32'hFC00_0000);          // DLC, EDL, BRS
        push_word(getw(3));                          // N
        push_word(fwd_d0);
        push_word(fwd_d1);
        $display("[%0t] PC: frame B forwarded back over USB as standard 0x123 (D0=0x%08h D1=0x%08h)", $time, fwd_d0, fwd_d1);

        tries = 0;
        do begin
            #1000ns;
            rd32(FSR, d);
            tries++;
        end while (d[30:24] == 0 && tries < 1500);

        if (d[30:24] == 0) begin
            errors++;
            $error("[%0t] frame C: RX FIFO-1 stayed empty after %0d polls: addr=0x%08h actual=0x%08h", $time, tries, FSR, d);
            rd32(SR,  d);  $display("DIAG SR  = 0x%08h  NORMAL=%0b BBSY=%0b ESTAT=%0b%0b", d, d[3], d[5], d[8], d[7]);
            rd32(ESR, d);  $display("DIAG ESR = 0x%08h  ACKER=%0b BERR=%0b STER=%0b FMER=%0b CRCER=%0b", d, d[4], d[3], d[2], d[1], d[0]);
        end
        else begin
            rxb = RB1_BASE + RX_STRIDE * d[21:16];
            rd32(rxb + 32'h0, id);  rd32(rxb + 32'h4, dlc);
            rd32(rxb + 32'h8, dw0); rd32(rxb + 32'hC, dw1);
            $display("[%0t] VIP: frame C in RX FIFO-1: FSR=0x%08h  ID=0x%08h DLC=0x%08h DW0=0x%08h DW1=0x%08h",
                     $time, d, id, dlc, dw0, dw1);
            chk("frame C ID (new ID)",  rxb + 32'h0, ID_A,    id);
            chk("frame C DLC",          rxb + 32'h4, DLC_W,   dlc & 32'hFFE0_0000);
            chk("frame C matched filter (1 = standard 0x123 -> RX FIFO-1)", rxb + 32'h4, 32'd1, {27'd0, dlc[20:16]});
            chk("frame C D0 (as read back by the PC)", rxb + 32'h8, fwd_d0, dw0);
            chk("frame C D1 (as read back by the PC)", rxb + 32'hC, fwd_d1, dw1);
            chk("frame C D0 (original payload)",       rxb + 32'h8, D0,     dw0);
            chk("frame C D1 (original payload)",       rxb + 32'hC, D1,     dw1);

            wr32(FSR, 32'h0080_0000);                // IRI_1: pop RX FIFO-1
        end
        chk("frame C must not reach USB channel A again", 32'h0, 32'd24, cap_n);
        end
        chk("frames_sent (bridge sent frame A, plus C when forwarding)", 32'h0, FORWARD_TEST ? 32'd2 : 32'd1, {16'd0, frames_sent});

        if (errors == 0) $display("PASS: A = PC->bridge->VIP, B = VIP->bridge->PC, C = B forwarded PC->bridge->VIP; both masters transmit and receive");
        else             $display("FAIL: %0d error(s)", errors);
        $finish;
    end

    // watchdog
    initial begin
        #10ms;
        $error("TIMEOUT: cap_n=%0d frames_sent=%0d", cap_n, frames_sent);
        $finish;
    end

endmodule
