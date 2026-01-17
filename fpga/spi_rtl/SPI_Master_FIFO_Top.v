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

module SPI_Master_FIFO_Top
    (
    input i_Rst_L,
    input i_Clk,

    // SPI Interface
    output SPI_CLK,
    input  SPI_MISO,
    output SPI_MOSI,
    output Sample_Debug,

    // Configuration
    input CPOL,
    input CPHA,
    input [7:0] CLKS_PER_HALF_BIT,
    input i_Begin_Exchange,

    // TX FIFO Interface (User writes to this)
    input [7:0] i_TX_Byte,
    input i_TX_Wr_En,
    output o_TX_Full,
    output [10:0] o_TX_FIFO_Count,
    output o_Busy,

    // RX FIFO Interface (User reads from this)
    output [7:0] o_RX_Byte,
    input i_RX_Rd_En,
    output o_RX_Empty
    );

    // Internal Signals
    wire [7:0] w_TX_FIFO_Data;
    wire w_TX_FIFO_Empty;
    reg w_TX_FIFO_Rd_En;

    // Core Signals
    reg [7:0] r_Core_TX_Byte;
    reg r_Core_TX_DataValid;
    wire w_Core_TX_Done;
    wire w_Core_RX_DataValid;
    wire [7:0] w_Core_RX_Byte;

    // State Machine
    localparam STATE_IDLE = 2'b00;
    localparam STATE_FETCH = 2'b01;
    localparam STATE_WAIT = 2'b10;
    localparam STATE_LATCH = 2'b11;

    reg [1:0] state;

    assign o_Busy = (state != STATE_IDLE) || (!w_TX_FIFO_Empty);

    // Latch to keep transaction active until FIFO is empty
    reg r_Tx_Active;
    always @(posedge i_Clk or negedge i_Rst_L) begin
        if (!i_Rst_L) 
            r_Tx_Active <= 0;
        else begin
            if (i_Begin_Exchange) 
                r_Tx_Active <= 1;
            else if (w_TX_FIFO_Empty && state == STATE_IDLE) 
                r_Tx_Active <= 0;
        end
    end

    // Instantiate SPI Master Core
    SPI_Master_Core u_SPI_Core (
        .i_Rst_L(i_Rst_L),
        .i_Clk(i_Clk),
        .SPI_CLK(SPI_CLK),
        .SPI_MISO(SPI_MISO),
        .SPI_MOSI(SPI_MOSI),
        .TX_Byte(r_Core_TX_Byte),
        .TX_DataValid(r_Core_TX_DataValid),
        .TX_Done(w_Core_TX_Done),
        .RX_DataValid(w_Core_RX_DataValid),
        .RX_Byte(w_Core_RX_Byte),
        .Sample_Debug(Sample_Debug),
        .CPOL(CPOL),
        .CPHA(CPHA),
        .CLKS_PER_HALF_BIT(CLKS_PER_HALF_BIT)
    );
    // Auto-reset TX FIFO when transaction ends (State machine finishes and FIFO is empty)
    wire w_TX_Flush = (state == STATE_WAIT) && w_Core_TX_Done && w_TX_FIFO_Empty;
    // Auto-reset RX FIFO when it becomes empty and is not being written to
    wire w_RX_Flush = o_RX_Empty && !w_Core_RX_DataValid;

    // Instantiate TX FIFO (1KB)
    Sync_FIFO #(.DEPTH(1024), .DATA_WIDTH(8)) u_TX_FIFO (
        .clk(i_Clk),
        .rst_n(i_Rst_L),
        .wr_en(i_TX_Wr_En),
        .din(i_TX_Byte),
        .full(o_TX_Full),
        .flush(w_TX_Flush),
        .count(o_TX_FIFO_Count),
        .rd_en(w_TX_FIFO_Rd_En),
        .dout(w_TX_FIFO_Data),
        .empty(w_TX_FIFO_Empty)
    );

    // Instantiate RX FIFO (1KB)
    Sync_FIFO #(.DEPTH(1024), .DATA_WIDTH(8)) u_RX_FIFO (
        .clk(i_Clk),
        .rst_n(i_Rst_L),
        .wr_en(w_Core_RX_DataValid), // Write when Core receives data
        .din(w_Core_RX_Byte),
        .full(), // Overflow handling not implemented in this simple version
        .flush(w_RX_Flush),
        .count(),
        .rd_en(i_RX_Rd_En),
        .dout(o_RX_Byte),
        .empty(o_RX_Empty)
    );

    // Control State Machine
    always @(posedge i_Clk or negedge i_Rst_L) begin
        if (!i_Rst_L) begin
            state <= STATE_IDLE;
            r_Core_TX_DataValid <= 0;
            r_Core_TX_Byte <= 0;
            w_TX_FIFO_Rd_En <= 0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    r_Core_TX_DataValid <= 0;
                    // If TX FIFO has data AND transaction is active, read it
                    if (!w_TX_FIFO_Empty && r_Tx_Active) begin
                        w_TX_FIFO_Rd_En <= 1;
                        state <= STATE_FETCH;
                    end
                end

                STATE_FETCH: begin
                    w_TX_FIFO_Rd_En <= 0;
                    state <= STATE_LATCH;
                end

                STATE_LATCH: begin
                    // Data from FIFO is available now (after latency)
                    r_Core_TX_Byte <= w_TX_FIFO_Data;
                    state <= STATE_WAIT;
                end

                STATE_WAIT: begin
                    // Wait for Core to finish transmission
                    r_Core_TX_DataValid <= 1;
                    if (w_Core_TX_Done) begin
                        r_Core_TX_DataValid <= 0;
                        state <= STATE_IDLE;
                    end
                end
            endcase
        end
    end

endmodule

// Simple Synchronous FIFO
module Sync_FIFO #(parameter DEPTH=1024, DATA_WIDTH=8) (
    input clk,
    input rst_n,
    input wr_en,
    input [DATA_WIDTH-1:0] din,
    output full,
    input flush,
    input rd_en,
    output reg [DATA_WIDTH-1:0] dout,
    output empty,
    output [$clog2(DEPTH):0] count
);

    (* ram_style = "block" *)
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [$clog2(DEPTH):0] wr_ptr = 0;
    reg [$clog2(DEPTH):0] rd_ptr = 0;

    assign count = wr_ptr - rd_ptr;
    assign empty = (wr_ptr == rd_ptr);
    assign full = (wr_ptr[$clog2(DEPTH)] != rd_ptr[$clog2(DEPTH)]) && 
                  (wr_ptr[$clog2(DEPTH)-1:0] == rd_ptr[$clog2(DEPTH)-1:0]);

    // Memory Access Block (Synchronous, No Reset) - Ensures BRAM Inference
    always @(posedge clk) begin
        if (!flush) begin
            if (wr_en && !full) begin
                mem[wr_ptr[$clog2(DEPTH)-1:0]] <= din;
            end
            if (rd_en && !empty) begin
                dout <= mem[rd_ptr[$clog2(DEPTH)-1:0]];
            end
        end
    end

    // Pointer Logic Block (Asynchronous Reset)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            if (flush) begin
                wr_ptr <= 0;
                rd_ptr <= 0;
            end else begin
                if (wr_en && !full) begin
                    wr_ptr <= wr_ptr + 1;
                end
                
                if (rd_en && !empty) begin
                    rd_ptr <= rd_ptr + 1;
                end
            end
        end
    end

endmodule
