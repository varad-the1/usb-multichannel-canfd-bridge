`timescale 1ns / 1ps

import axi_vip_pkg::*;
import axi_block_axi_vip_0_0_pkg::*;
import axi_block_axi_vip_0_1_pkg::*;

module tb();

    int tx_tries;
bit [31:0] writer_read_data;
bit [31:0] reader_read_data;

    bit m_axi_aclk  = 0;
    bit axi_aresetn = 0;
    bit [31:0] gpio_rtl_0;

    bit [31:0] read_data_axi;
    bit [31:0] rx_id, rx_dlc, rx_dw0, rx_dw1;

    int tries;

    axi_block_axi_vip_0_0_mst_t read_agent;
    axi_block_axi_vip_0_1_mst_t write_agent;

    xil_axi_resp_t read_resp;
    xil_axi_resp_t write_resp;
    xil_axi_prot_t prot = 0;

    event configuration_done;

    // Reset
    initial begin
        axi_aresetn = 0;
        #10000ns;
        axi_aresetn = 1;
    end

    // Clock
    always #5ns m_axi_aclk = !m_axi_aclk;

    // DUT
    axi_block_wrapper DUT (
        .clk              (m_axi_aclk),
        .reset_rtl_0      (axi_aresetn),
        .gpio_rtl_0_tri_o (gpio_rtl_0)
    );

    // Read-only VIP
    initial begin
        read_agent = new(
            "read agent",
            DUT.axi_block_i.axi_vip_0.inst.IF
        );

        read_agent.set_agent_tag("READ VIP");
        read_agent.set_verbosity(0);
        read_agent.start_master();
    end

    // Write-only VIP
    initial begin
        write_agent = new(
            "write agent",
            DUT.axi_block_i.axi_vip_1.inst.IF
        );

        write_agent.set_agent_tag("WRITE VIP");
        write_agent.set_verbosity(0);
        write_agent.start_master();
    end

    // =========================================================
    // WRITER: configure CAN FD and transmit one frame
    // =========================================================
    initial begin
        wait (axi_aresetn == 1'b1);
        #2000ns;

        // BRPR
        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_0008, prot, 32'h0000_0003, write_resp
        );

        // BTR
        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_000C, prot, 32'h0000_030E, write_resp
        );

        // Loopback mode
        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_0004, prot, 32'h0000_0002, write_resp
        );

        // Configure acceptance filter 0
        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_00E0, prot, 32'h0000_0000, write_resp
        );

        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_0A00, prot, 32'h0000_0000, write_resp
        );

        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_0A04, prot, 32'h0000_0000, write_resp
        );

        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_00E0, prot, 32'h0000_0001, write_resp
        );

        // Enable CAN FD
        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_0000, prot, 32'h0000_0002, write_resp
        );

        #20000ns;

        // Signal that configuration is complete.
        -> configuration_done;

        $display("[%0t] Writer: CAN FD configured", $time);

        // TX ID
        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_0100, prot, 32'h2460_0000, write_resp
        );

        // TX DLC: 8-byte CAN FD frame
        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_0104, prot, 32'h8800_0000, write_resp
        );

        // Payload
        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_0108, prot, 32'h1122_3344, write_resp
        );

        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_010C, prot, 32'h5566_7788, write_resp
        );

        $display("[%0t] Writer: TX buffer prepared", $time);

        // Request transmission from TX Buffer 0
        write_agent.AXI4LITE_WRITE_BURST(
            32'h44A0_0090, prot, 32'h0000_0001, write_resp
        );

        $display("[%0t] Writer: TX request issued", $time);

        // Poll TX request until it clears
        tx_tries = 0;

do begin
    #5000ns;

    write_agent.AXI4LITE_READ_BURST(
        32'h44A0_0090,
        prot,
        writer_read_data,
        read_resp
    );

    $display(
        "[%0t] TX Request Register = 0x%08h",
        $time,
        writer_read_data
    );

    tx_tries++;
end while (writer_read_data[0] && tx_tries < 100);

if (writer_read_data[0])
    $error("TX Buffer 0 request never cleared");
else
    $display("TX Buffer 0 request cleared");
    end
    // =========================================================
    // READER: independently monitor and verify RX FIFO
    // =========================================================
    initial begin
        // Wait until controller configuration has completed.
        @configuration_done;

        $display("[%0t] Reader: Monitoring RX FIFO", $time);

        // Poll RX FIFO-0 fill level.
        tries = 0;

        do begin
            #1000ns;

            read_agent.AXI4LITE_READ_BURST(
                32'h44A0_00E8, prot, read_data_axi, read_resp
            );

            tries++;
        end while (read_data_axi[14:8] == 0 && tries < 100);

        $display(
            "[%0t] Reader: RX FIFO FSR = 0x%08h",
            $time, read_data_axi
        );

        if (read_data_axi[14:8] == 0) begin
            $error("Reader: RX FIFO never filled");
        end
        else begin
            // Read received ID
            read_agent.AXI4LITE_READ_BURST(
                32'h44A0_2100, prot, rx_id, read_resp
            );

            // Read received DLC
            read_agent.AXI4LITE_READ_BURST(
                32'h44A0_2104, prot, rx_dlc, read_resp
            );

            // Read received payload
            read_agent.AXI4LITE_READ_BURST(
                32'h44A0_2108, prot, rx_dw0, read_resp
            );

            read_agent.AXI4LITE_READ_BURST(
                32'h44A0_210C, prot, rx_dw1, read_resp
            );

            $display("RX ID  = 0x%08h", rx_id);
            $display("RX DLC = 0x%08h", rx_dlc);
            $display("RX DW0 = 0x%08h", rx_dw0);
            $display("RX DW1 = 0x%08h", rx_dw1);

            // Verify ID, DLC, and payload.
            if (
                rx_id  === 32'h2460_0000 &&
                rx_dlc === 32'h8800_0000 &&
                rx_dw0 === 32'h1122_3344 &&
                rx_dw1 === 32'h5566_7788
            ) begin
                $display("PASS: Received frame matches transmitted frame");
            end
            else begin
                $error("FAIL: Received frame does not match expected frame");
            end

            // Pop RX FIFO entry.
            // This write must be performed by the writer VIP because
            // the reader VIP is intended to be read-only.
            write_agent.AXI4LITE_WRITE_BURST(
                32'h44A0_00E8, prot, 32'h0000_0080, write_resp
            );
        end
    end

endmodule