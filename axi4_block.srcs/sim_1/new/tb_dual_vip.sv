`timescale 1ns / 1ps

// Dual-VIP CAN FD loopback bench.
//   axi_vip_1 (WRITE_ONLY, S01_AXI): configures CAN, writes scratch, sends TX frame, pops RX FIFO
//   axi_vip_0 (READ_ONLY,  S00_AXI): reads scratch back, polls SR/FSR, reads RX frame
// Scratch word = axi_gpio_1 DATA register (all-outputs, 32-bit, readable back).

module tb_dual_vip;

    import axi_vip_pkg::*;
    import axi_block_axi_vip_0_0_pkg::*;
    import axi_block_axi_vip_0_1_pkg::*;   // note: IP for axi_vip_1 is named ..._vip_0_1

    localparam bit [31:0] CAN_BASE  = 32'h44A0_0000;
    localparam bit [31:0] GPIO_BASE = 32'h4001_0000;   // Address Editor: SEG_axi_gpio_1_Reg

    localparam bit [31:0] SRR   = CAN_BASE + 32'h000;
    localparam bit [31:0] MSR   = CAN_BASE + 32'h004;
    localparam bit [31:0] BRPR  = CAN_BASE + 32'h008;
    localparam bit [31:0] BTR   = CAN_BASE + 32'h00C;
    localparam bit [31:0] ESR   = CAN_BASE + 32'h014;   // PG223: ACKER = bit 4
    localparam bit [31:0] SR    = CAN_BASE + 32'h018;
    localparam bit [31:0] TRR   = CAN_BASE + 32'h090;
    localparam bit [31:0] AFR   = CAN_BASE + 32'h0E0;
    localparam bit [31:0] FSR   = CAN_BASE + 32'h0E8;
    localparam bit [31:0] AFMR0 = CAN_BASE + 32'hA00;
    localparam bit [31:0] AFIR0 = CAN_BASE + 32'hA04;
    localparam bit [31:0] TB0_ID  = CAN_BASE + 32'h100;
    localparam bit [31:0] TB0_DLC = CAN_BASE + 32'h104;
    localparam bit [31:0] TB0_DW0 = CAN_BASE + 32'h108;
    localparam bit [31:0] TB0_DW1 = CAN_BASE + 32'h10C;
    localparam bit [31:0] RX_ID   = CAN_BASE + 32'h2100;
    localparam bit [31:0] RX_STRIDE = 32'h48;        // RB*-ID of buffer n = 0x2100 + 0x48*n
    localparam bit [31:0] RX_DLC  = CAN_BASE + 32'h2104;
    localparam bit [31:0] RX_DW0  = CAN_BASE + 32'h2108;
    localparam bit [31:0] RX_DW1  = CAN_BASE + 32'h210C;

    localparam int N_SCRATCH = 4;
    localparam bit [31:0] SCRATCH_VAL [N_SCRATCH] = '{
        32'hA5A5_5A5A, 32'h0123_4567, 32'hFEDC_BA98, 32'hDEAD_BEEF };

    // ---------------------------------------------------------
    // Frames under test, sent back to back (once each, not endlessly).
    //   F_ID : 11-bit ID for standard, 29-bit ID for extended
    //   F_FD : 0 = classic CAN, 1 = CAN FD (up to 64 bytes). Data byte k = 0x11*(k+1): 11 22 33 44 55 66 77 88 ...
    // ---------------------------------------------------------
    localparam int N_FRAMES = 2;
    localparam bit [31:0] F_ID  [N_FRAMES] = '{ 32'h0000_0123, 32'h18FE_CA00 };
    localparam bit        F_EXT [N_FRAMES] = '{ 1'b0,          1'b1 };
    localparam bit        F_FD  [N_FRAMES] = '{ 1'b0,          1'b0 };
    localparam int        F_LEN [N_FRAMES] = '{ 8,             8 };

    function automatic bit [7:0] fbyte(input int k);
        return 8'((k + 1) * 8'h11);
    endfunction

    // TB*-ID / RB*-ID layout (PG223): [31:21]=ID[28:18] [20]=SRR/RTR/RRS [19]=IDE [18:1]=ID[17:0] [0]=RTR/RRS
    function automatic bit [31:0] f_idreg(input int f);
        if (F_EXT[f]) return {F_ID[f][28:18], 1'b1, 1'b1, F_ID[f][17:0], 1'b0};
        else          return {F_ID[f][10:0], 21'b0};
    endfunction

    function automatic int len2dlc(input int len);
        case (len)
            12: return 9;  16: return 10; 20: return 11; 24: return 12;
            32: return 13; 48: return 14; 64: return 15;
            default: return (len > 8) ? 15 : len;
        endcase
    endfunction

    // TB*-DLC layout (PG223): [31:28]=DLC [27]=EDL/FDF [26]=BRS (0) ...
    function automatic bit [31:0] f_dlcreg(input int f);
        return {4'(len2dlc(F_LEN[f])), F_FD[f], 27'b0};
    endfunction

    // data word w: bytes 4w..4w+3, first byte in the MSB (zero padded past the length)
    function automatic bit [31:0] f_word(input int f, input int w);
        bit [31:0] v = 0;
        for (int k = 4*w; k < 4*w + 4; k++)
            v = (v << 8) | ((k < F_LEN[f]) ? 32'(fbyte(k)) : 32'h0);
        return v;
    endfunction

    function automatic bit [31:0] f_mask(input int f, input int w);
        int nb = F_LEN[f] - 4*w;
        if (nb >= 4) return 32'hFFFF_FFFF;
        return ~(32'hFFFF_FFFF >> (8*nb));
    endfunction

    // ---------------------------------------------------------
    // Clock / reset / DUT
    // ---------------------------------------------------------
    bit         clk = 0;
    bit         resetn = 0;               // reset_rtl_0 is active-low
    wire [31:0] gpio_out;

    always #5ns clk = ~clk;

    initial begin
        resetn = 0;
        #10000ns;
        resetn = 1;
    end

    axi_block_wrapper DUT (
        .clk                (clk),
        .reset_rtl_0        (resetn),
        .gpio_rtl_0_tri_o   (gpio_out)
    );

    // CAN bus edge logger: <time_ps> <can_phy_tx> <can_phy_rx>, for offline frame decoding
    integer canlog;
    initial canlog = $fopen("can_edges.txt", "w");
    always @(DUT.axi_block_i.canfd_0.can_phy_tx or DUT.axi_block_i.canfd_0.can_phy_rx)
        $fdisplay(canlog, "%0t %b %b", $time, DUT.axi_block_i.canfd_0.can_phy_tx,
                  DUT.axi_block_i.canfd_0.can_phy_rx);
    final begin $fflush(canlog); $fclose(canlog); end

    // ---------------------------------------------------------
    // Agents
    // ---------------------------------------------------------
    axi_block_axi_vip_0_0_mst_t rd_agent;   // axi_vip_0, read-only
    axi_block_axi_vip_0_1_mst_t wr_agent;   // axi_vip_1, write-only

    // ---------------------------------------------------------
    // Shared sync state (flags only; no resp/data shared)
    // ---------------------------------------------------------
    int  errors      = 0;
    bit  cfg_done    = 0;   // writer: CAN configured
    int  scr_written = 0;   // writer: scratch words written so far
    int  scr_checked = 0;   // reader: scratch words verified so far
    int  tx_req_cnt  = 0;   // writer: frames requested so far
    int  rx_chk_cnt  = 0;   // reader: frames checked so far

    // ---------------------------------------------------------
    // Helpers (automatic: resp/data are per-call locals)
    // ---------------------------------------------------------
    task automatic wr32(input bit [31:0] addr, input bit [31:0] data);
        xil_axi_resp_t resp;
        wr_agent.AXI4LITE_WRITE_BURST(addr, 0, data, resp);
        if (resp !== XIL_AXI_RESP_OKAY) begin
            errors++;
            $error("[%0t] WR addr=0x%08h resp expected=OKAY actual=%0d", $time, addr, resp);
        end
    endtask

    task automatic rd32(input bit [31:0] addr, output bit [31:0] data);
        xil_axi_resp_t resp;
        bit [31:0] d;
        rd_agent.AXI4LITE_READ_BURST(addr, 0, d, resp);
        if (resp !== XIL_AXI_RESP_OKAY) begin
            errors++;
            $error("[%0t] RD addr=0x%08h resp expected=OKAY actual=%0d", $time, addr, resp);
        end
        data = d;
    endtask

    task automatic chk(input bit [31:0] addr, input bit [31:0] exp, input bit [31:0] act);
        if (act !== exp) begin
            errors++;
            $error("[%0t] MISMATCH addr=0x%08h expected=0x%08h actual=0x%08h", $time, addr, exp, act);
        end
    endtask

    // ---------------------------------------------------------
    // Agent start
    // ---------------------------------------------------------
    initial begin
        rd_agent = new("rd agent", DUT.axi_block_i.axi_vip_0.inst.IF);
        rd_agent.set_agent_tag("READ VIP");
        rd_agent.set_verbosity(0);
        rd_agent.start_master();

        wr_agent = new("wr agent", DUT.axi_block_i.axi_vip_1.inst.IF);
        wr_agent.set_agent_tag("WRITE VIP");
        wr_agent.set_verbosity(0);
        wr_agent.start_master();
    end

    // ---------------------------------------------------------
    // Writer thread (axi_vip_1)
    // ---------------------------------------------------------
    task automatic writer();
        bit [31:0] unused;
        wait (resetn === 1'b1);
        #2000ns;

        wr32(BRPR,  32'h0000_0003);
        wr32(BTR,   32'h0000_030E);
        // NOTE: internal loopback (MSR = 0x2) disabled. The BD now shorts canfd_0/can_phy_tx
        // to can_phy_rx, so the frame goes out the PHY pins and back in (external loopback).
        // wr32(MSR,   32'h0000_0002);  // internal loopback (old)
        wr32(MSR,   32'h0000_0000);     // normal mode, external loopback via tx->rx short
        wr32(AFR,   32'h0000_0000);     // disable filters while editing
        wr32(AFMR0, 32'h0000_0000);     // mask 0 = accept all
        wr32(AFIR0, 32'h0000_0000);
        wr32(AFR,   32'h0000_0001);     // UAF0 = 1
        wr32(SRR,   32'h0000_0002);     // CEN
        #20000ns;
        cfg_done = 1;
        $display("[%0t] Writer: CAN configured", $time);

        // Scratch words; wait for reader to verify each before overwriting
        for (int i = 0; i < N_SCRATCH; i++) begin
            wr32(GPIO_BASE, SCRATCH_VAL[i]);
            scr_written = i + 1;
            wait (scr_checked == i + 1);
        end

        for (int f = 0; f < N_FRAMES; f++) begin
            wr32(TB0_ID,  f_idreg(f));
            wr32(TB0_DLC, f_dlcreg(f));
            for (int w = 0; w < (F_LEN[f] + 3) / 4; w++)
                wr32(TB0_DW0 + 4*w, f_word(f, w));
            wr32(TRR, 32'h0000_0001);
            tx_req_cnt = f + 1;
            $display("[%0t] Writer: frame %0d requested (ID=0x%08h %s %s, %0d bytes)", $time, f, F_ID[f],
                     F_EXT[f] ? "extended" : "standard", F_FD[f] ? "CAN FD" : "CAN", F_LEN[f]);

            wait (rx_chk_cnt == f + 1);
            wr32(FSR, 32'h0000_0080);       // IRI: pop RX FIFO-0
            $display("[%0t] Writer: frame %0d RX FIFO popped", $time, f);
        end
    endtask

    // ---------------------------------------------------------
    // Reader thread (axi_vip_0)
    // ---------------------------------------------------------
    task automatic reader();
        bit [31:0] d, id, dlc, dw, rxb;
        int tries;

        wait (cfg_done);
        $display("[%0t] Reader: started", $time);

        for (int i = 0; i < N_SCRATCH; i++) begin
            wait (scr_written == i + 1);
            rd32(GPIO_BASE, d);
            chk(GPIO_BASE, SCRATCH_VAL[i], d);
            chk(GPIO_BASE, SCRATCH_VAL[i], gpio_out);   // pin must follow the register
            scr_checked = i + 1;
        end

        for (int f = 0; f < N_FRAMES; f++) begin
            wait (tx_req_cnt == f + 1);

            // Poll RX FIFO-0 fill level (FSR[14:8]) while also sampling SR
            tries = 0;
            do begin
                #1000ns;
                rd32(SR,  d);
                rd32(FSR, d);
                tries++;
            end while (d[14:8] == 0 && tries < 600);

            if (d[14:8] == 0) begin
                errors++;
                $error("[%0t] frame %0d: RX FIFO empty after %0d polls: addr=0x%08h expected fill>=1 actual=0x%08h",
                       $time, f, tries, FSR, d);
                // Diagnostics (PG223): SR NORMAL/LBACK/ESTAT/BBSY, ESR ACKER, TRR still pending?
                rd32(SR,  d);  $display("DIAG SR  = 0x%08h  CONFIG=%0b LBACK=%0b NORMAL=%0b BBSY=%0b ESTAT=%0b%0b ERRWRN=%0b",
                                        d, d[0], d[1], d[3], d[5], d[8], d[7], d[6]);
                rd32(ESR, d);  $display("DIAG ESR = 0x%08h  ACKER=%0b BERR=%0b STER=%0b FMER=%0b CRCER=%0b",
                                        d, d[4], d[3], d[2], d[1], d[0]);
                rd32(TRR, d);  $display("DIAG TRR = 0x%08h  (bit0=1: TX buffer 0 still pending)", d);
            end
            else begin
                // FIFO mode: the oldest message sits at 0x2100 + 0x48*RI, RI = FSR[5:0] (PG223)
                rxb = RX_ID + RX_STRIDE * d[5:0];
                rd32(rxb + 32'h0, id);
                rd32(rxb + 32'h4, dlc);
                $display("[%0t] frame %0d: FSR=0x%08h (RI=%0d)  RX ID=0x%08h DLC=0x%08h", $time, f, d, d[5:0], id, dlc);
                chk(rxb + 32'h0, f_idreg(f),  id);
                chk(rxb + 32'h4, f_dlcreg(f), dlc & 32'hFFFF_0000);   // [15:0] = RX timestamp, varies
                for (int w = 0; w < (F_LEN[f] + 3) / 4; w++) begin
                    rd32(rxb + 32'h8 + 4*w, dw);
                    $display("[%0t] frame %0d: RX DW%0d=0x%08h", $time, f, w, dw);
                    chk(rxb + 32'h8 + 4*w, f_word(f, w) & f_mask(f, w), dw & f_mask(f, w));
                end
            end
            rx_chk_cnt = f + 1;
        end
    endtask

    // ---------------------------------------------------------
    // Latency measurement: write on axi_vip_1 -> visible on axi_vip_0
    // Waveforms to probe (all on the VIP master ports, DUT.axi_block_i):
    //   axi_vip_1: m_axi_awvalid & m_axi_awready  (write address accepted)  [t_aw]
    //   axi_vip_0: m_axi_rvalid  & m_axi_rready & m_axi_rdata == new value  [t_r]
    // Optional third: gpio_rtl_0_tri_o (register actually changed)          [t_gpio]
    // ---------------------------------------------------------
    localparam int N_LAT = 8;
    wire wr_aw_hs = DUT.axi_block_i.axi_vip_1.m_axi_awvalid & DUT.axi_block_i.axi_vip_1.m_axi_awready;
    wire wr_b_hs  = DUT.axi_block_i.axi_vip_1.m_axi_bvalid  & DUT.axi_block_i.axi_vip_1.m_axi_bready;
    wire rd_ar_hs = DUT.axi_block_i.axi_vip_0.m_axi_arvalid & DUT.axi_block_i.axi_vip_0.m_axi_arready;
    wire rd_r_hs  = DUT.axi_block_i.axi_vip_0.m_axi_rvalid  & DUT.axi_block_i.axi_vip_0.m_axi_rready;
    wire [31:0] rd_rdata = DUT.axi_block_i.axi_vip_0.m_axi_rdata;

    bit  lat_active = 0;
    bit [31:0] lat_val;
    time t_aw, t_b, t_gpio, t_ar_last, t_ar_match, t_r_match;
    bit  lat_matched;

    always @(posedge clk) if (lat_active) begin
        if (wr_aw_hs) t_aw = $time;
        if (wr_b_hs)  t_b  = $time;
        if (rd_ar_hs) t_ar_last = $time;
        if (rd_r_hs && rd_rdata === lat_val && !lat_matched && t_aw != 0) begin
            lat_matched = 1; t_ar_match = t_ar_last; t_r_match = $time;
        end
    end
    always @(gpio_out) if (lat_active && gpio_out === lat_val && t_aw != 0) t_gpio = $time;

    task automatic latency_test();
        time sum_gpio = 0, sum_r = 0, worst_r = 0;
        for (int i = 0; i < N_LAT; i++) begin
            bit [31:0] v = 32'hC0DE_0000 + i + 1;
            t_aw = 0; t_b = 0; t_gpio = 0; t_ar_last = 0; lat_matched = 0;
            lat_val = v; lat_active = 1;
            fork
                wr32(GPIO_BASE, v);                       // writer (axi_vip_1)
                begin                                     // reader (axi_vip_0), back-to-back polling
                    bit [31:0] d;
                    int n = 0;
                    do begin rd32(GPIO_BASE, d); n++; end while (d !== v && n < 200);
                end
            join
            lat_active = 0;
            if (!lat_matched) begin
                errors++;
                $error("[%0t] LAT addr=0x%08h expected=0x%08h never seen on read side", $time, GPIO_BASE, v);
            end else begin
                $display("LAT[%0d] val=0x%08h  AW->GPIO=%0t  AW->R(match)=%0t  B->R(match)=%0t",
                         i, v, t_gpio - t_aw, t_r_match - t_aw, t_r_match - t_b);
                sum_gpio += t_gpio - t_aw; sum_r += t_r_match - t_aw;
                if (t_r_match - t_aw > worst_r) worst_r = t_r_match - t_aw;
            end
            #500ns;
        end
        $display("LAT summary: avg AW->GPIO=%0t  avg AW->R(match)=%0t  worst AW->R(match)=%0t",
                 sum_gpio / N_LAT, sum_r / N_LAT, worst_r);
    endtask

    // ---------------------------------------------------------
    // External-loopback responder (second CAN node, testbench only)
    // can_phy_tx and can_phy_rx are one net in the BD (the "bus"). A lone core gets no ACK
    // and never stores its own frames (PG223, Normal Mode), so this model:
    //   1) decodes the frame being sent (CAN FD, standard ID, BRS=0) and drives the ACK slot dominant
    //   2) after the frame ends, replays the same bit stream as a frame from "another node"
    //      (ACK slot left to the core), so it lands in the RX FIFO
    // Limitations: standard 11-bit ID, FDF=1, BRS=0 (single bit rate); a run of 5 equal bits
    // exactly at the end of the data field is not handled.
    // ---------------------------------------------------------
    localparam bit  EXT_RESPONDER = 1;
    localparam time BT = 1000ns;           // nominal bit time, measured from can_edges.txt (BRPR=3, BTR=0x30E)
    wire can_bus = DUT.axi_block_i.canfd_0_can_phy_tx;   // same net as can_phy_rx
    bit  bus_drive;                                      // force source (force cannot use automatic vars)

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
            @(negedge can_bus);                 // SOF
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
                                $display("[%0t] RESP: BRS=1 not supported, no ACK", $time);
                            end else
                                $display("[%0t] RESP: SOF at %0t, %s %s, DLC=%0d (%0d bytes)", $time, t0,
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
                        if (!bad && crc_end >= 0 && n == crc_end && ack_idx < 0) begin
                            ack_idx = i + 2 + ((run == 5) ? 1 : 0);    // optional stuff bit, CRC delimiter, then ACK slot
                        end
                        if (ack_idx >= 0)
                            $display("[%0t] RESP: ACK slot = bit %0d", $time, ack_idx);
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
            force DUT.axi_block_i.canfd_0.can_phy_rx = 1'b0;
            #(BT);
            release DUT.axi_block_i.canfd_0.can_phy_rx;
            $display("[%0t] RESP: ACK driven", $time);

            // --- 2) wait out ACK delimiter + EOF(7) + IFS(3), then replay as another node ---
            wait_until(t0 + (ack_idx + 12 + 3)*BT);
            for (int j = 0; j < ack_idx; j++) begin
                bus_drive = raw[j];
                force DUT.axi_block_i.canfd_0.can_phy_rx = bus_drive;
                #(BT);
            end
            release DUT.axi_block_i.canfd_0.can_phy_rx;   // ACK slot: the core drives it
            $display("[%0t] RESP: replayed frame, ACK slot left to core", $time);
            wait_until($time + 9*BT);                     // ACK slot, delimiter, EOF; the core's IFS follows
        end
    endtask

    initial if (EXT_RESPONDER) begin
        wait (resetn === 1'b1);
        ext_responder();
    end

    // ---------------------------------------------------------
    // Main
    // ---------------------------------------------------------
    initial begin
        wait (resetn === 1'b1);
        #100ns;   // agents are created in a separate initial at t=0

        fork
            writer();
            reader();
        join

        // Writer pop verification via reader
        begin
            bit [31:0] d;
            #2000ns;
            rd32(FSR, d);
            chk(FSR, 32'h0, d[14:8]);
        end

        latency_test();

        if (errors == 0) $display("PASS: dual-VIP scratch and CAN loopback payload match");
        else             $display("FAIL: %0d error(s)", errors);
        $finish;
    end

    // Watchdog
    initial begin
        #20ms;
        $error("TIMEOUT: testbench did not finish");
        $finish;
    end

endmodule
