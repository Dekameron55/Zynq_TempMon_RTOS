/******************************************************************************
*
* Copyright (C) 2026
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
******************************************************************************/

import axi_vip_pkg::*;
import design_1_axi_vip_0_0_pkg::*;

module axi_spi_master_tb();


    // -----------------------------------------------------------------------
    // Signals
    // -----------------------------------------------------------------------
    bit aclk = 0;
    bit aresetn = 0;

    // SPI Physical Signals (Must be exposed in Block Design Wrapper)
    wire spi_clk;
    wire spi_mosi;
    wire spi_miso;
    wire spi_cs_n;
    wire spi_irq;
    wire sample_debug;

    // -----------------------------------------------------------------------
    // DUT Instantiation
    // -----------------------------------------------------------------------
    // Note: Ensure your Block Design (design_1) has external ports for SPI and Interrupts
    design_1_wrapper DUT (
        .aclk(aclk),           // Verify port name in design_1_wrapper.v (e.g. aclk_0)
        .aresetn(aresetn),     // Verify port name in design_1_wrapper.v (e.g. aresetn_0)
        .sclk(spi_clk),     // Verify port name in design_1_wrapper.v
        .SPI_CS_N(spi_cs_n),
        .dbg(spi_irq),
        .sd(sample_debug)
    );

    // Loopback for self-test: Connect MISO directly to MOSI
    assign spi_miso = spi_mosi;

    // -----------------------------------------------------------------------
    // AXI VIP Agent
    // -----------------------------------------------------------------------
    // If this type is unknown, ensure the AXI VIP in the Block Design is configured as 'Master'.
    design_1_axi_vip_0_0_mst_t agent;

    // -----------------------------------------------------------------------
    // Register Map
    // -----------------------------------------------------------------------
    localparam bit [31:0] ADDR_CTRL_REG     = 32'h00;
    localparam bit [31:0] ADDR_STATUS_REG   = 32'h04;
    localparam bit [31:0] ADDR_CLK_DIV_REG  = 32'h08;
    localparam bit [31:0] ADDR_TX_DATA_REG  = 32'h0C;
    localparam bit [31:0] ADDR_RX_DATA_REG  = 32'h10;
    localparam bit [31:0] ADDR_IER_REG      = 32'h14;
    localparam bit [31:0] ADDR_ISR_REG      = 32'h18;
    localparam bit [31:0] ADDR_CS_REG       = 32'h1C;
    localparam bit [31:0] ADDR_VERSION_REG  = 32'h20;
    localparam bit [31:0] ADDR_START_REG    = 32'h24;
    localparam bit [31:0] ADDR_RESET_REG    = 32'h28;

    // -----------------------------------------------------------------------
    // Clock Generation
    // -----------------------------------------------------------------------
    always #5 aclk = ~aclk; // 100 MHz

    // -----------------------------------------------------------------------
    // Main Test Process
    // -----------------------------------------------------------------------
    initial begin
        bit [31:0] version_data;
        // 1. Initialize and Start AXI VIP Agent
        // The path below assumes standard Vivado Block Design hierarchy:
        // wrapper -> block_design -> axi_vip_inst -> interface
        // You may need to adjust 'design_1_i' to match the instance name in design_1_wrapper.v
        agent = new("master_vip_agent", DUT.design_1_i.axi_vip_0.inst.IF);
        agent.start_master();

        // 2. System Reset
        $display("[%0t] Resetting System...", $time);
        aresetn = 0;
        #200;
        aresetn = 1;
        #200;

        // Check IP Version
        $display("[%0t] Checking IP Version...", $time);
        axi_read(ADDR_VERSION_REG, version_data);
        $display("IP Version: 0x%h", version_data);
        if (version_data != 32'h00010100) $error("Version Mismatch! Expected 0x00010000");

        // 3. Configure SPI Master
        $display("[%0t] Configuring SPI Master...", $time);
        
        // Set Clock Divider (e.g., 4 => 12.5 MHz SPI Clock)
        axi_write(ADDR_CLK_DIV_REG, 32'h04);
        
        // Set Control (CPOL=0, CPHA=0)
        axi_write(ADDR_CTRL_REG, 32'h00);

        // Enable Interrupts (Bit 1: Exchange End)
        axi_write(ADDR_IER_REG, 32'h02);

        // 4. Perform First Loopback Test
        $display("[%0t] Starting First Loopback Test (4 Bytes)...", $time);
        
        // Assert Chip Select (Active Low)
        axi_write(ADDR_CS_REG, 32'h00);

        // Fill TX FIFO
        axi_write(ADDR_TX_DATA_REG, 32'hDE);
        axi_write(ADDR_TX_DATA_REG, 32'hAD);
        axi_write(ADDR_TX_DATA_REG, 32'hBE);
        axi_write(ADDR_TX_DATA_REG, 32'hEF);


        // Start the exchange by writing 1 to START REG
        axi_write(ADDR_START_REG, 32'h01);

        // Wait for Transaction Completion (via Interrupt)
        wait_for_interrupt();

        // Deassert Chip Select
        axi_write(ADDR_CS_REG, 32'h01);

        // Clear Interrupt (Write 1 to Clear reg 1)
        axi_write(ADDR_ISR_REG, 32'h02);

        // Verify Received Data for first exchange
        check_rx_fifo(32'hDE);
        check_rx_fifo(32'hAD);
        check_rx_fifo(32'hBE);
        check_rx_fifo(32'hEF);

        // Verify FIFO is empty before next exchange
        check_status_empty();
        $display("[%0t] First exchange successful.", $time);
        #200;

        // 5. Perform Second Loopback Test with different data
        $display("[%0t] Starting Second Loopback Test (2 Bytes)...", $time);

        // Assert Chip Select
        axi_write(ADDR_CS_REG, 32'h00);

        axi_write(ADDR_TX_DATA_REG, 32'h12);
        axi_write(ADDR_TX_DATA_REG, 32'h34);

        // Start the exchange
        axi_write(ADDR_CTRL_REG, 32'h04);
        axi_write(ADDR_START_REG, 32'h01);

        wait_for_interrupt();

        // Deassert Chip Select
        axi_write(ADDR_CS_REG, 32'h01);

        axi_write(ADDR_ISR_REG, 32'h02);

        // Verify Received Data for second exchange
        check_rx_fifo(32'h12);
        check_rx_fifo(32'h34);

        // Verify FIFO is empty at the end
        check_status_empty();

        $display("---------------------------------------------------");
        $display("   TEST PASSED: AXI SPI Master Loopback Successful ");
        $display("---------------------------------------------------");
        $finish;
    end

    // -----------------------------------------------------------------------
    // Helper Tasks
    // -----------------------------------------------------------------------

    // Task: Write Register using AXI VIP
    task axi_write(input bit [31:0] addr, input bit [31:0] data);
        xil_axi_resp_t resp;
        begin
            agent.AXI4LITE_WRITE_BURST(addr, 0, data, resp);
            if (resp != XIL_AXI_RESP_OKAY) begin
                $error("AXI Write Error! Addr: 0x%h, Resp: %0d", addr, resp);
            end
        end
    endtask

    // Task: Read Register using AXI VIP
    task axi_read(input bit [31:0] addr, output bit [31:0] data);
        xil_axi_resp_t resp;
        begin
            agent.AXI4LITE_READ_BURST(addr, 0, data, resp);
            if (resp != XIL_AXI_RESP_OKAY) begin
                $error("AXI Read Error! Addr: 0x%h, Resp: %0d", addr, resp);
            end
        end
    endtask

    // Task: Check RX FIFO Data
    task check_rx_fifo(input bit [31:0] expected_byte);
        bit [31:0] rx_data;
        bit [31:0] status;
        
        begin
            // Check if RX FIFO is empty before reading
            axi_read(ADDR_STATUS_REG, status);
            if (status[1]) begin // reg 1 is RX_Empty
                $error("Error: RX FIFO is Empty, expected data 0x%h", expected_byte);
            end

            // Read Data
            axi_read(ADDR_RX_DATA_REG, rx_data);
            
            // Verify
            if ((rx_data & 32'hFF) !== expected_byte) begin
                $error("Data Mismatch! Expected: 0x%h, Received: 0x%h", expected_byte, rx_data & 32'hFF);
            end else begin
                $display("[%0t] Received Correct Data: 0x%h", $time, rx_data & 32'hFF);
            end
        end
    endtask

    // Task: Check Status Empty
    task check_status_empty();
        bit [31:0] status;
        begin
            axi_read(ADDR_STATUS_REG, status);
            if (!status[1]) begin
                $error("Error: RX FIFO should be empty but is not!");
            end
        end
    endtask

    // Task: Wait for Interrupt
    task wait_for_interrupt();
        integer timeout = 100000; // Cycles
        begin
            $display("[%0t] Waiting for Interrupt...", $time);
            while (spi_irq == 0 && timeout > 0) begin
                @(posedge aclk);
                timeout--;
            end
            
            if (timeout == 0) begin
                $error("Timeout waiting for SPI Interrupt!");
            end else begin
                $display("[%0t] Interrupt Asserted.", $time);
            end
        end
    endtask

endmodule
