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

`timescale 1ns/1ps

module SPI_Master_FIFO_Top_tb;

    // Inputs
    reg i_Rst_L;
    reg i_Clk;
    reg CPOL;
    reg CPHA;
    reg [7:0] CLKS_PER_HALF_BIT;
    reg [7:0] i_TX_Byte;
    reg i_TX_Wr_En;
    reg i_RX_Rd_En;
    reg i_Begin_Exchange;

    // Outputs
    wire SPI_CLK;
    wire SPI_MISO;
    wire SPI_MOSI;
    wire Sample_Debug;
    wire o_TX_Full;
    wire [7:0] o_RX_Byte;
    wire o_RX_Empty;

    // Loopback: Connect MOSI to MISO
    assign SPI_MISO = SPI_MOSI;

    // Instantiate the Unit Under Test (UUT)
    SPI_Master_FIFO_Top uut (
        .i_Rst_L(i_Rst_L), 
        .i_Clk(i_Clk), 
        .SPI_CLK(SPI_CLK), 
        .SPI_MISO(SPI_MISO), 
        .SPI_MOSI(SPI_MOSI), 
        .Sample_Debug(Sample_Debug), 
        .CPOL(CPOL), 
        .CPHA(CPHA), 
        .CLKS_PER_HALF_BIT(CLKS_PER_HALF_BIT), 
        .i_Begin_Exchange(i_Begin_Exchange),
        .i_TX_Byte(i_TX_Byte), 
        .i_TX_Wr_En(i_TX_Wr_En), 
        .o_TX_Full(o_TX_Full), 
        .o_RX_Byte(o_RX_Byte), 
        .i_RX_Rd_En(i_RX_Rd_En), 
        .o_RX_Empty(o_RX_Empty)
    );

    // Clock Generation (100 MHz)
    always #5 i_Clk = ~i_Clk;

    initial begin
        // Initialize Inputs
        i_Clk = 0;
        i_Rst_L = 0;
        CPOL = 0;
        CPHA = 0;
        CLKS_PER_HALF_BIT = 4; // 12.5 MHz SPI Clock
        i_TX_Byte = 0;
        i_TX_Wr_En = 0;
        i_RX_Rd_En = 0;
        i_Begin_Exchange = 0;

        // Setup VCD Dump
        $dumpfile("dump_top.vcd");
        $dumpvars(0, SPI_Master_FIFO_Top_tb);

        // Reset
        #20 i_Rst_L = 1;
        #20;

        $display("Test Started: Filling TX FIFO with 4 bytes");

        // Write 4 bytes to TX FIFO: 0xA1, 0xB2, 0xC3, 0xD4
        write_fifo(8'hA1);
        write_fifo(8'hB2);
        write_fifo(8'hC3);
        write_fifo(8'hD4);

        // Trigger the exchange
        @(posedge i_Clk);
        i_Begin_Exchange <= 1;
        @(posedge i_Clk);
        i_Begin_Exchange <= 0;

        // Wait for transmission to complete
        // 4 bytes * 8 bits * 8 clocks/bit * 10ns = ~2.5us
        // Add some margin
        #5000;

        $display("Transmission should be complete. Checking RX FIFO...");

        // Read back and verify
        read_and_check(8'hA1);
        read_and_check(8'hB2);
        read_and_check(8'hC3);
        read_and_check(8'hD4);

        if (o_RX_Empty)
            $display("Test Passed: RX FIFO is empty as expected.");
        else
            $display("Test Failed: RX FIFO not empty.");

        #100;
        $finish;
    end

    // Task to write to FIFO
    task write_fifo(input [7:0] data);
    begin
        @(posedge i_Clk);
        i_TX_Byte <= data;
        i_TX_Wr_En <= 1;
        @(posedge i_Clk);
        i_TX_Wr_En <= 0;
    end
    endtask

    // Task to read from FIFO and check data
    task read_and_check(input [7:0] expected);
    begin
        if (o_RX_Empty) begin
            $display("Error: RX FIFO Empty when expecting 0x%h", expected);
        end else begin
            @(posedge i_Clk);
            i_RX_Rd_En <= 1;
            @(posedge i_Clk);
            i_RX_Rd_En <= 0;
            // Wait for data to appear (Sync FIFO latency)
            @(posedge i_Clk); 
            #1;
            if (o_RX_Byte === expected)
                $display("Success: Received 0x%h", o_RX_Byte);
            else
                $display("Error: Expected 0x%h, Received 0x%h", expected, o_RX_Byte);
        end
    end
    endtask

endmodule