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

module SPI_Master_Core
    (
    input i_Rst_L,
    input i_Clk,

    input CPOL,
    input CPHA,
    input [7:0] CLKS_PER_HALF_BIT,
    
    output SPI_CLK,
    input  SPI_MISO,
    output SPI_MOSI,

    input [7:0] TX_Byte,
    input TX_DataValid,
    output TX_Done,

    output RX_DataValid,
    output [7:0] RX_Byte,

    output Sample_Debug

    );



reg SPI_tick;
reg [7:0] SPI_tick_count; // Fixed width to match CLKS_PER_HALF_BIT

reg [3:0] TxBitCount;
reg [3:0] RxBitCount;

reg [7:0] TX_Byte_reg;

reg r_SPI_Clk = 0;
reg r_SPI_MOSI = 0;
reg r_TX_Done = 0;
reg r_RX_DataValid = 0;
reg [7:0] r_RX_Byte = 0;
reg r_Sample_Debug = 0;

assign SPI_CLK = r_SPI_Clk ^ CPOL;
assign SPI_MOSI = r_SPI_MOSI;
assign TX_Done = r_TX_Done;
assign RX_DataValid = r_RX_DataValid;
assign RX_Byte = r_RX_Byte;
assign Sample_Debug = r_Sample_Debug;

reg TrailingEdge = 0;
reg LeadingEdge = 0;

always @(posedge i_Clk or negedge i_Rst_L)
begin
    if (~i_Rst_L)
    begin
        r_SPI_Clk <= 0; // Default to 0, XORed with CPOL at output
        SPI_tick <= 0;
        SPI_tick_count <= 0;
        r_SPI_MOSI <= 0;
        TxBitCount <= 7;
        RxBitCount <= 7; // MSB first
        TX_Byte_reg <= 0;
        r_RX_Byte <= 0;
        r_TX_Done <= 0;
        r_RX_DataValid <= 0;
        r_Sample_Debug <= 0;
        LeadingEdge <= 0;
        TrailingEdge <= 0;
    end
    else
    begin
        r_TX_Done <= 0;
        r_RX_DataValid <= 0;
        r_Sample_Debug <= 0;

        // Latch TX Byte at start
        if (TX_DataValid && SPI_tick_count == 0 && TxBitCount == 7)
        begin
            TX_Byte_reg <= TX_Byte;
            // For CPHA=0, setup first bit immediately
            if (!CPHA) r_SPI_MOSI <= TX_Byte[7];
        end

        // Clock Generation
        if (TX_DataValid)
        begin
            if (SPI_tick_count == CLKS_PER_HALF_BIT - 1)
            begin
                LeadingEdge <= 1'b1;
                TrailingEdge <= 1'b0;
                SPI_tick_count <= SPI_tick_count + 1;
                SPI_tick <= 1;
            end
            else if (SPI_tick_count == CLKS_PER_HALF_BIT * 2 - 1)
            begin
                TrailingEdge <= 1'b1;
                LeadingEdge <= 1'b0;
                SPI_tick_count <= 0;
                SPI_tick <= 1;
            end
            else
            begin
                SPI_tick <= 0;
                SPI_tick_count <= SPI_tick_count + 1;
                LeadingEdge <= 0;
                TrailingEdge <= 0;
            end
        end
        else
        begin
            SPI_tick_count <= 0;
            r_SPI_Clk <= 0;
            TxBitCount <= 7;
            RxBitCount <= 7;
            SPI_tick <= 0;
            r_TX_Done <= 0;
        end

        // Data Logic (Enabled by SPI_tick)
        if (SPI_tick)
        begin
            if (TrailingEdge || LeadingEdge)
            begin
                r_SPI_Clk <= ~r_SPI_Clk;
            end
            
            // Update Bit Counter and Done Status (Always on Trailing Edge)
            if (TrailingEdge)
            begin
                if (TxBitCount > 0)
                begin
                    TxBitCount <= TxBitCount - 1;
                end
                else
                begin
                    TxBitCount <= 7;
                    r_TX_Done <= 1; // Pulse Done
                end
            end
            else
            begin
                r_TX_Done <= 0;
            end

            // MOSI Logic
            if (CPHA && LeadingEdge)
            begin
                r_SPI_MOSI <= TX_Byte_reg[TxBitCount];
            end
            else if (!CPHA && TrailingEdge)
            begin
                if (TxBitCount > 0)
                    r_SPI_MOSI <= TX_Byte_reg[TxBitCount - 1];
            end

            // MISO Logic
            if ((~CPHA && LeadingEdge) || (CPHA && TrailingEdge))
            begin
                r_Sample_Debug <= 1;
                r_RX_Byte[RxBitCount] <= SPI_MISO;
                
                if (RxBitCount == 0)
                begin
                    r_RX_DataValid <= 1;
                    RxBitCount <= 7;
                end
                else
                begin
                    r_RX_DataValid <= 0;
                    RxBitCount <= RxBitCount - 1;
                end
            end
            else
            begin
                r_Sample_Debug <= 0;
            end
        end
        else
        begin
            r_Sample_Debug <= 0;
        end
    end
end

endmodule