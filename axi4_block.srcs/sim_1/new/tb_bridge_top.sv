`timescale 1ns / 1ps

// Two-channel bench, EXTERNAL loopback, bridge is the only AXI master.
//
//   can_frames.txt   -> channel 0 -> CAN FD core 0 -> bus 0 -+
//   can_frames_2.txt -> channel 1 -> CAN FD core 1 -> bus 1 -+-> second node replays each frame
//   all frames go in over USB channel B, come back over USB channel A tagged with their channel number,
//   and are written to ONE output file, one line per frame, with the channel in front:
//        CH<n> 0x<ID> <length> <data bytes>
//
// The first frame sent after power-up is a configuration frame (word 0 = 0xFFFFFFFF); data sent before it is dropped.
//
// For every pair (frame p of channel 0, frame p of channel 1) both frames are sent back to back, so the two
// channels run at the same time; the replies are then matched by their channel word.
//
// Bridge cell settings assumed: NUM_CH = 2.
// BASE0 = 0x44A00000 (core 0 at 0x44A00000, core 1 at 0x44A10000).

module tb_bridge_top;

    parameter string IN_FILE0  = "D:/internship/axi4_block_canfd/can_frames.txt";
    parameter string IN_FILE1  = "D:/internship/axi4_block_canfd/can_frames_2.txt";
    parameter string OUT_FILE  = "D:/internship/axi4_block_canfd/tb_io/channels_out.txt";
    parameter int    MAX_FRAMES = 0;          // frames per channel to send, 0 = all of them
    localparam int   MAX_MSG   = 512;

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
    reg  [7:0] ub [0:1023];
    integer    ub_n = 0, ub_i = 0;
    wire       ftb_rd_n;
    wire [7:0] ftb_data  = (ub_i < ub_n) ? ub[ub_i] : 8'h00;
    wire       ftb_rxf_n = !(ub_i < ub_n);
    always @(posedge ftb_rd_n) if (resetn) #2 ub_i = ub_i + 1;

    wire       fta_wr_n, fta_rd_n, fta_oe_n, ftb_wr_n;
    wire [7:0] fta_data;
    reg  [7:0] cap [0:1023];
    integer    cap_n = 0;
    always @(posedge ft_clk) if (fta_wr_n === 1'b0) begin cap[cap_n] = fta_data; cap_n = cap_n + 1; end

    // ---------------------------------------------------------
    // Two CAN buses: each core's TX ANDed with its own second node (dominant 0 wins)
    // ---------------------------------------------------------
    wire can0_tx, can1_tx;
    wire resp0, resp1;                               // second nodes' outputs, 1 = recessive
    wire can0_bus = can0_tx & resp0;
    wire can1_bus = can1_tx & resp1;

    can_second_node node0 (.bus(can0_bus), .line(resp0), .go(resetn));
    can_second_node node1 (.bus(can1_bus), .line(resp1), .go(resetn));

    wire [31:0] frames_sent;                         // 16 bits per channel
    wire [1:0]  cfg_done;                            // per channel: configuration sequence finished

    axi_block_wrapper DUT (
        .CAN_INTERFACE_0_rx (can0_bus),
        .CAN_INTERFACE_0_tx (can0_tx),
        .CAN_INTERFACE_1_rx (can1_bus),
        .CAN_INTERFACE_1_tx (can1_tx),
        .clk                (clk),
        .reset_rtl_0        (resetn),
        .frames_sent_0      (frames_sent),
        .cfg_done_0         (cfg_done),
        .ft_clk_0           (ft_clk),
        .fta_data_0         (fta_data),
        .fta_oe_n_0         (fta_oe_n),
        .fta_rd_n_0         (fta_rd_n),
        .fta_txe_n_0        (1'b0),
        .fta_wr_n_0         (fta_wr_n),
        .ftb_data_0         (ftb_data),
        .ftb_rd_n_0         (ftb_rd_n),
        .ftb_rxf_n_0        (ftb_rxf_n),
        .ftb_wr_n_0         (ftb_wr_n)
    );

    int errors = 0;

    // ---------------------------------------------------------
    // PC side: push words over USB channel B / read them back from channel A
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
    // File-driven test
    //   input line:  <ID hex> <length in bytes> <data bytes hex>      ("//" lines are ignored)
    // ---------------------------------------------------------
    bit [31:0] m_id   [2][MAX_MSG];
    int        m_len  [2][MAX_MSG];
    bit [7:0]  m_data [2][MAX_MSG][64];
    int        n_msg  [2];
    bit [31:0] cfg_id  [2][32];                       // CFG lines of each file
    bit [31:0] cfg_val [2][32];
    int        n_cfg  [2];

    function automatic bit [31:0] hex_of(input string t);
        bit [31:0] v = 0;
        int k = 0;
        if (t.len() > 2 && t[0] == "0" && (t[1] == "x" || t[1] == "X")) k = 2;
        for (; k < t.len(); k++) begin
            if      (t[k] >= "0" && t[k] <= "9") v = (v << 4) | (t[k] - "0");
            else if (t[k] >= "a" && t[k] <= "f") v = (v << 4) | (t[k] - "a" + 10);
            else if (t[k] >= "A" && t[k] <= "F") v = (v << 4) | (t[k] - "A" + 10);
        end
        return v;
    endfunction

    task automatic load_messages(input int ch, input string path);
        int fd, c, n;
        string t;
        n  = 0;
        n_cfg[ch] = 0;
        fd = $fopen(path, "r");
        if (fd == 0) begin $error("cannot open %s", path); $finish; end
        while (!$feof(fd) && n < MAX_MSG) begin
            c = $fscanf(fd, "%s", t);
            if (c != 1) break;
            if (t.len() >= 2 && t.substr(0,1) == "//") begin
                void'($fgets(t, fd));
                continue;
            end
            if (t == "CFG" || t == "cfg") begin               // CFG <register id> <value>
                void'($fscanf(fd, "%s", t));  cfg_id[ch][n_cfg[ch]]  = hex_of(t);
                void'($fscanf(fd, "%s", t));  cfg_val[ch][n_cfg[ch]] = hex_of(t);
                n_cfg[ch]++;
                continue;
            end
            m_id[ch][n] = hex_of(t);
            void'($fscanf(fd, "%d", m_len[ch][n]));
            for (int k = 0; k < m_len[ch][n]; k++) begin
                void'($fscanf(fd, "%s", t));
                m_data[ch][n][k] = hex_of(t);
            end
            n++;
        end
        $fclose(fd);
        n_msg[ch] = n;
        $display("channel %0d: loaded %0d frames from %s", ch, n, path);
    endtask

    function automatic bit [31:0] id_word(input bit [31:0] id);        // PG223 TX/RX ID word
        if (id > 32'h7FF) return {id[28:18], 1'b1, 1'b1, id[17:0], 1'b0};
        else              return {id[10:0], 21'd0};
    endfunction

    function automatic bit [31:0] id_of_word(input bit [31:0] w);
        if (w[19]) return {3'd0, w[31:21], w[18:1]};
        else       return {21'd0, w[31:21]};
    endfunction

    function automatic bit [3:0] dlc_code(input int len);
        if (len <= 8)  return len;
        if (len <= 12) return 9;
        if (len <= 16) return 10;
        if (len <= 20) return 11;
        if (len <= 24) return 12;
        if (len <= 32) return 13;
        if (len <= 48) return 14;
        return 15;
    endfunction

    function automatic bit [31:0] data_word(input int ch, input int m, input int w);   // first byte = MSB
        bit [31:0] v = 0;
        for (int k = 0; k < 4; k++) v = (v << 8) | ((4*w + k < m_len[ch][m]) ? m_data[ch][m][4*w + k] : 8'h00);
        return v;
    endfunction

    task automatic push_frame(input int ch, input int m);
        int nw;
        nw = (m_len[ch][m] + 3) / 4;
        push_word(ch);
        push_word(id_word(m_id[ch][m]));
        push_word({dlc_code(m_len[ch][m]), (m_len[ch][m] > 8) ? 1'b1 : 1'b0, 27'd0});
        push_word(nw);
        for (int w = 0; w < nw; w++) push_word(data_word(ch, m, w));
    endtask

    int total [2];
    int bad   [2];
    int fd_out;

    // check the frame that starts at word 'pos' of the capture against the frame sent on its channel
    task automatic check_received(input int pos, input int idx, input bit sent0, input bit sent1);
        bit [31:0] g0, g1, g2, g3, gw, ew;
        bit [4:0]  exp_hdr;
        bit        ok, c_id, c_hdr, c_n, c_dat, c_sent;
        int        ch, m, nw, exp_nw;
        g0 = getw(pos);  g1 = getw(pos+1);  g2 = getw(pos+2);  g3 = getw(pos+3);
        nw = g3;
        ch = (g0 == 0) ? 0 : (g0 == 1) ? 1 : -1;

        // output file: channel in front, same layout as the input files
        $fwrite(fd_out, "CH%0d 0x%0h %0d ", g0, id_of_word(g1), 4*nw);
        for (int k = 0; k < 4*nw; k++) $fwrite(fd_out, "%02h ", cap[4*(pos+4) + 4*(k/4) + (3 - k%4)]);
        $fwrite(fd_out, "%s", "\n");
        $fflush(fd_out);

        if (ch < 0) begin
            bad[0]++;
            $display("[%0t] MISMATCH: frame with unknown channel word %0d", $time, g0);
            return;
        end
        m      = idx;
        c_sent = (ch == 0) ? sent0 : sent1;
        total[ch]++;
        if (!c_sent) begin
            bad[ch]++;
            $display("[%0t] MISMATCH CH%0d: received a frame that was not sent", $time, ch);
            return;
        end
        exp_nw  = (m_len[ch][m] + 3) / 4;
        exp_hdr = {dlc_code(m_len[ch][m]), (m_len[ch][m] > 8) ? 1'b1 : 1'b0};
        c_id  = (g1 == id_word(m_id[ch][m]));
        c_hdr = (g2[31:27] == exp_hdr);
        c_n   = (nw == exp_nw);
        c_dat = 1;
        for (int w = 0; w < exp_nw; w++) begin
            gw = getw(pos + 4 + w);
            ew = data_word(ch, m, w);
            if (gw !== ew) c_dat = 0;
        end
        ok = c_id && c_hdr && c_n && c_dat;
        if (!ok) begin
            bad[ch]++;
            $display("[%0t] MISMATCH CH%0d frame %0d (ID 0x%0h, %0d bytes): id=%0b hdr=%0b n=%0b data=%0b | got ID=0x%08h DLC=0x%08h N=%0d",
                     $time, ch, m, m_id[ch][m], m_len[ch][m], c_id, c_hdr, c_n, c_dat, g1, g2, nw);
        end
    endtask

    initial begin
        int rounds, p, tries, pos, sent_words, fr_words;
        int act;
        bit [7:0] act_mask;
        bit s0, s1;
        wait (resetn === 1'b1);
        #1000ns;
        total[0] = 0; total[1] = 0; bad[0] = 0; bad[1] = 0;
        load_messages(0, IN_FILE0);
        load_messages(1, IN_FILE1);
        rounds = (n_msg[0] > n_msg[1]) ? n_msg[0] : n_msg[1];
        if (MAX_FRAMES > 0 && MAX_FRAMES < rounds) rounds = MAX_FRAMES;

        // ---- before the configuration frame: a data frame must be dropped, nothing may reach the CAN bus ----
        push_frame(0, 0);
        #200us;
        if (cap_n != 0 || cfg_done != 2'b00) begin
            errors++;
            $display("[%0t] MISMATCH: frame sent before configuration got through (cap_n=%0d cfg_done=%b)", $time, cap_n, cfg_done);
        end else
            $display("[%0t] frame sent before configuration was dropped, as required", $time);
        ub_n = 0; ub_i = 0; cap_n = 0;

        // ---- configuration frames from the input files (CFG lines): file 0 first, then file 1 ----
        //   frame = 0xFFFFFFFF, M, then M x (register id, value).  Nothing is built in: every value comes from the files.
        act = 0;
        for (int cf = 0; cf < 2; cf++) begin
            if (n_cfg[cf] > 0) begin
                push_word(32'hFFFF_FFFF);  push_word(n_cfg[cf]);
                for (int k = 0; k < n_cfg[cf]; k++) begin
                    push_word(cfg_id[cf][k]);  push_word(cfg_val[cf][k]);
                    if (cfg_id[cf][k] == 32'h0000_0000) act = cfg_val[cf][k];
                end
                $display("config frame from file %0d: %0d register writes", cf, n_cfg[cf]);
            end
        end
        act_mask = (act >= 8) ? 8'hFF : ((8'd1 << act) - 8'd1);
        begin : wait_cfg
            int w = 0;
            while ((cfg_done & act_mask) != act_mask && w < 5000) begin #1000ns; w++; end
            if (act == 0 || (cfg_done & act_mask) != act_mask) begin
                errors++;
                $display("[%0t] MISMATCH: cores not configured, cfg_done=%b (active channels from the files: %0d)", $time, cfg_done, act);
            end else
                $display("[%0t] configuration applied, %0d cores configured (cfg_done=%b)", $time, act, cfg_done);
        end
        ub_n = 0; ub_i = 0; cap_n = 0;

        fd_out = $fopen(OUT_FILE, "w");
        $fwrite(fd_out, "// channel, ID, length (bytes), data -- received over USB channel A from %s and %s%s", IN_FILE0, IN_FILE1, "\n");

        for (p = 0; p < rounds; p++) begin
            s0 = (p < n_msg[0]);
            s1 = (p < n_msg[1]);
            ub_n = 0; ub_i = 0; cap_n = 0;                  // fresh buffers per pair
            sent_words = 0;
            if (s0) begin push_frame(0, p); sent_words += 4 + (m_len[0][p] + 3) / 4; end
            if (s1) begin push_frame(1, p); sent_words += 4 + (m_len[1][p] + 3) / 4; end

            tries = 0;
            while (cap_n < 4*sent_words && tries < 30000) begin #1000ns; tries++; end
            #3000ns;

            pos = 0;
            while (pos*4 + 16 <= cap_n) begin
                fr_words = 4 + getw(pos + 3);
                if (pos + fr_words > cap_n/4) break;
                check_received(pos, p, s0, s1);
                pos += fr_words;
            end
            if (cap_n != 4*sent_words) begin
                $display("[%0t] MISMATCH pair %0d: expected %0d bytes back, got %0d", $time, p, 4*sent_words, cap_n);
                bad[0]++;
            end
            if ((p + 1) % 10 == 0) $display("[%0t] %0d / %0d pairs done, mismatches ch0=%0d ch1=%0d", $time, p + 1, rounds, bad[0], bad[1]);
        end
        $fclose(fd_out);

        $display("RESULT CH0: %0d received, %0d wrong", total[0], bad[0]);
        $display("RESULT CH1: %0d received, %0d wrong", total[1], bad[1]);
        $display("Output: %s", OUT_FILE);
        if (bad[0] == 0 && bad[1] == 0 && errors == 0) $display("PASS: every frame came back unchanged on its own channel");
        else                                            $display("FAIL: ch0 %0d wrong, ch1 %0d wrong", bad[0], bad[1]);
        $finish;
    end

    // watchdog
    initial begin
        #20000ms;
        $error("TIMEOUT: cap_n=%0d", cap_n);
        $finish;
    end

endmodule

// ---------------------------------------------------------
// Second CAN node. Listens to the core's frame, drives the ACK slot dominant, then replays
// the same bits as a frame from "another node" (the core drives the ACK slot of the replay).
// Handles standard/extended IDs, classic CAN and CAN FD with BRS = 0. A run of five equal
// bits exactly at the end of the data field is not handled in CAN FD frames.
// ---------------------------------------------------------
module can_second_node (
    input  wire bus,
    output bit  line,
    input  wire go
);
    initial line = 1'b1;

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
            wait (bus === 1'b1);
            @(negedge bus);                 // SOF
            if (bus !== 1'b0) continue;
            t0 = $time;
            raw.delete();
            i = 0; n = 0; run = 0; ack_idx = -1; dlc = 0; bytes = 0; bad = 0; prev = 0;
            ide = 0; fdf = 0; brs = 0; f_pos = 14; dlc_lo = 1000; data_end = -1; crc_end = -1;

            // --- decode while sampling at mid-bit. n counts destuffed bits (SOF = 0) ---
            //   standard: SOF, ID(11), RTR/RRS, IDE, FDF/r0 ...      extended: ... IDE, ID(18), RTR/RRS, FDF/r1 ...
            while (!bad && (ack_idx < 0 || i < ack_idx)) begin
                wait_until(t0 + i*BT + BT/2);
                b = bus;
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
            line = 1'b0;
            #(BT);
            line = 1'b1;
            $display("[%0t] BUS: ACK driven", $time);

            // --- 2) wait out ACK delimiter + EOF(7) + IFS(3), then replay as another node ---
            wait_until(t0 + (ack_idx + 12 + 3)*BT);
            for (int j = 0; j < ack_idx; j++) begin
                line = raw[j];
                #(BT);
            end
            line = 1'b1;                   // ACK slot: the core drives it
            $display("[%0t] BUS: replayed frame, ACK slot left to the core", $time);
            wait_until($time + 9*BT);           // ACK slot, delimiter, EOF; the core's IFS follows
        end
    endtask

    initial begin
        wait (go === 1'b1);
        ext_responder();
    end
endmodule
