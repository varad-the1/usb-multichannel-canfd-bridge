`timescale 1ns/1ps

module tb;

    // =========================================================
    // AXI VIP packages
    // =========================================================

    import axi_vip_pkg::*;
    import axi_block_axi_vip_0_0_pkg::*;


    // =========================================================
    // Clock / Reset
    // =========================================================

    logic clk;
    logic reset;

    wire gpio_out;


    // 100 MHz clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end


    // =========================================================
    // DUT
    // =========================================================

    axi_block_wrapper dut (
        .clk         (clk),
        .reset_rtl_0 (reset),
        .gpio_rtl_0_tri_o  (gpio_out)
    );


    // =========================================================
    // AXI VIP Master
    // =========================================================

    axi_block_axi_vip_0_0_mst_t master_agent;


    // =========================================================
    // Test
    // =========================================================

    initial begin

        xil_axi_ulong addr;
        xil_axi_prot_t prot;
        xil_axi_data_t read_data;
        xil_axi_resp_t resp;


        // -----------------------------------------------------
        // Initial reset
        // -----------------------------------------------------

        reset = 1;

        repeat (10)
            @(posedge clk);

        reset = 0;

        repeat (10)
            @(posedge clk);


        // -----------------------------------------------------
        // Create AXI VIP master
        // -----------------------------------------------------

        master_agent = new(
            "AXI Master",
            dut.axi_block_i.axi_vip_0.inst.IF
        );

        master_agent.start_master();


        // =====================================================
        // READ GPIO0
        // Address: 0x4000_0000
        // =====================================================

        $display("");
        $display("==========================================");
        $display(" AXI READ");
        $display(" Address : 0x40000000");
        $display("==========================================");

        master_agent.AXI4LITE_READ_BURST(
            32'h4000_0000,
            0,
            read_data,
            resp
        );

        $display("Read data : 0x%08h", read_data);
        $display("Response  : %0d", resp);


        // =====================================================
        // WRITE GPIO1
        // Address: 0x4001_0000
        // =====================================================

        $display("");
        $display("==========================================");
        $display(" AXI WRITE");
        $display(" Address : 0x40010000");
        $display(" Data    : 0x%08h", read_data);
        $display("==========================================");

        master_agent.AXI4LITE_WRITE_BURST(
            32'h4001_0000,
            0,
            read_data,
            resp
        );


        // -----------------------------------------------------
        // Wait for GPIO output
        // -----------------------------------------------------

        repeat (10)
            @(posedge clk);


        // =====================================================
        // CHECK OUTPUT
        // =====================================================

        $display("");
        $display("==========================================");
        $display(" GPIO CHECK");
        $display(" Expected : %b", read_data[0]);
        $display(" Actual   : %b", gpio_out);
        $display("==========================================");


        if (gpio_out === read_data[0]) begin

            $display("");
            $display("*************** PASS ***************");
            $display("");

        end
        else begin

            $display("");
            $display("*************** FAIL ***************");
            $display("");

            $error(
                "GPIO mismatch! Expected %b, got %b",
                read_data[0],
                gpio_out
            );

        end


        #100;

        $finish;

    end

endmodule