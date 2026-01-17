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

`timescale 1ns / 1ps

module axi_spi_master #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 6,
    parameter [31:0]  C_IP_VERSION       = 32'h00010300
)
(
    // AXI4-Lite Slave Interface
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 S_AXI_ACLK CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET S_AXI_ARESETN" *)
    input  wire                               S_AXI_ACLK,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 S_AXI_ARESETN RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire                               S_AXI_ARESETN,
    
    // Write Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]      S_AXI_AWADDR,
    input  wire [2:0]                         S_AXI_AWPROT,
    input  wire                               S_AXI_AWVALID,
    output wire                               S_AXI_AWREADY,
    
    // Write Data Channel
    input  wire [C_S_AXI_DATA_WIDTH-1:0]      S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]  S_AXI_WSTRB,
    input  wire                               S_AXI_WVALID,
    output wire                               S_AXI_WREADY,
    
    // Write Response Channel
    output wire [1:0]                         S_AXI_BRESP,
    output wire                               S_AXI_BVALID,
    input  wire                               S_AXI_BREADY,
    
    // Read Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]      S_AXI_ARADDR,
    input  wire [2:0]                         S_AXI_ARPROT,
    input  wire                               S_AXI_ARVALID,
    output wire                               S_AXI_ARREADY,
    
    // Read Data Channel
    output wire [C_S_AXI_DATA_WIDTH-1:0]      S_AXI_RDATA,
    output wire [1:0]                         S_AXI_RRESP,
    output wire                               S_AXI_RVALID,
    input  wire                               S_AXI_RREADY,

    // SPI Physical Interface
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 SPI_CLK CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 0" *)
    output wire                               SPI_CLK,
    input  wire                               SPI_MISO,
    output wire                               SPI_MOSI,
    output wire                               SPI_CS_N,
    output wire                               Sample_Debug,

    // Interrupt
    output wire                               interrupt
);

    // Register Address Map
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_CTRL_REG     = 5'h00; // RW: [0] CPOL, [1] CPHA
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_STATUS_REG   = 5'h04; // RO: [0] TX_Full, [1] RX_Empty
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_CLK_DIV_REG  = 5'h08; // RW: [7:0] CLKS_PER_HALF_BIT
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_TX_DATA_REG  = 5'h0C; // WO: Write to push to TX FIFO
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_RX_DATA_REG  = 5'h10; // RO: Read to pop from RX FIFO
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_IER_REG      = 5'h14; // RW: Interrupt Enable
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_ISR_REG      = 5'h18; // RW: Interrupt Status (W1C)
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_CS_REG       = 5'h1C; // RW: Chip Select (Active Low)
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_VERSION_REG  = 6'h20; // RO: IP Version
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_START_REG    = 6'h24; // WO: Start Transaction, [0] BEGIN_EXCHANGE (self-clearing)
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_RESET_REG    = 6'h28; // WO: Software Reset


    // Internal Registers
    reg [1:0]  reg_control;      // CPOL, CPHA
    reg [7:0]  reg_clk_div;      // CLKS_PER_HALF_BIT
    reg [1:0]  reg_ier;          // [0] TX_Half_Empty, [1] Exchange_End
    reg        reg_cs;           // Chip Select
    reg        reg_begin_exchange; // Pulse to start transaction
    reg        reg_soft_reset;     // Pulse to reset core
    
    // AXI Internal Signals
    reg axi_awready;
    reg axi_wready;
    reg [1:0] axi_bresp;
    reg axi_bvalid;
    reg axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1:0] axi_rdata;
    reg [1:0] axi_rresp;
    reg axi_rvalid;
    reg r_reading_fifo;

    // SPI Core Signals
    reg  [7:0] spi_tx_byte;
    reg        spi_tx_wr_en;
    wire       spi_tx_full;
    wire [7:0] spi_rx_byte;
    reg        spi_rx_rd_en;
    wire       spi_rx_empty;
    wire [10:0] spi_tx_count;
    wire        spi_busy;

    wire w_core_rst_n = S_AXI_ARESETN & ~reg_soft_reset;

    // Instantiation
    SPI_Master_FIFO_Top u_spi_top (
        .i_Rst_L(w_core_rst_n),
        .i_Clk(S_AXI_ACLK),
        .SPI_CLK(SPI_CLK),
        .SPI_MISO(SPI_MISO),
        .SPI_MOSI(SPI_MOSI),
        .Sample_Debug(Sample_Debug),
        .CPOL(reg_control[0]),
        .CPHA(reg_control[1]),
        .i_Begin_Exchange(reg_begin_exchange),
        .CLKS_PER_HALF_BIT(reg_clk_div),
        .i_TX_Byte(spi_tx_byte),
        .i_TX_Wr_En(spi_tx_wr_en),
        .o_TX_Full(spi_tx_full),
        .o_TX_FIFO_Count(spi_tx_count),
        .o_Busy(spi_busy),
        .o_RX_Byte(spi_rx_byte),
        .i_RX_Rd_En(spi_rx_rd_en),
        .o_RX_Empty(spi_rx_empty)
    );

    // I/O Connections assignments
    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;

    // Interrupt Logic
    // ISR[0] = TX FIFO Half Empty (Level Sensitive) - Useful for high speed refilling
    // ISR[1] = Exchange End (Falling Edge of Busy) - Useful for simple exchanges
    
    reg isr_done;
    reg spi_busy_d;
    wire isr_tx_half;
    
    assign isr_tx_half = (spi_tx_count <= 11'd512);
    
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            spi_busy_d <= 1'b0;
            isr_done   <= 1'b0;
        end else begin
            spi_busy_d <= spi_busy;
            // Detect Falling Edge of Busy
            if (spi_busy_d && !spi_busy) begin
                isr_done <= 1'b1;
            end
            // Clear on Write (handled in Write Logic below)
            if (S_AXI_AWVALID && S_AXI_WVALID && axi_awready && (S_AXI_AWADDR[C_S_AXI_ADDR_WIDTH-1:0] == ADDR_ISR_REG) && S_AXI_WDATA[1]) begin
                isr_done <= 1'b0;
            end
        end
    end

    wire [1:0] isr_vector;
    assign isr_vector = {isr_done, isr_tx_half};
    assign interrupt = |(isr_vector & reg_ier);

    //-------------------------------------------------------------------------
    // Write Channel Logic
    //-------------------------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_awready <= 1'b0;
            axi_wready  <= 1'b0;
            axi_bvalid  <= 1'b0;
            axi_bresp   <= 2'b0;
            reg_control <= 2'b00; // Default Mode 0
            reg_clk_div <= 8'd4;  // Default Divider
            reg_ier     <= 2'b00; // Interrupts Disabled
            spi_tx_wr_en <= 1'b0;
            spi_tx_byte  <= 8'b0;
            reg_cs       <= 1'b1; // Default Inactive (High)
            reg_begin_exchange <= 1'b0;
            reg_soft_reset <= 1'b0;
        end else begin
            // Handshake Logic
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID) begin
                axi_awready <= 1'b1;
                axi_wready  <= 1'b1;
            end else begin
                axi_awready <= 1'b0;
                axi_wready  <= 1'b0;
            end

            // Write Response
            if (axi_awready && axi_wready && ~axi_bvalid) begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b0; // OKAY
            end else if (S_AXI_BREADY && axi_bvalid) begin
                axi_bvalid <= 1'b0;
            end

            // Register Write Logic
            spi_tx_wr_en <= 1'b0; // Default pulse low
            reg_begin_exchange <= 1'b0; // It's a self-clearing pulse
            reg_soft_reset <= 1'b0;

            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID) begin
                case (S_AXI_AWADDR[C_S_AXI_ADDR_WIDTH-1:0])
                    ADDR_CTRL_REG: begin
                        reg_control <= S_AXI_WDATA[1:0];
                    end
                    ADDR_CLK_DIV_REG: begin
                        reg_clk_div <= S_AXI_WDATA[7:0];
                    end
                    ADDR_TX_DATA_REG: begin
                        spi_tx_byte  <= S_AXI_WDATA[7:0];
                        spi_tx_wr_en <= 1'b1; // Pulse write enable
                    end
                    ADDR_IER_REG: begin
                        reg_ier <= S_AXI_WDATA[1:0];
                    end
                    ADDR_ISR_REG: begin
                        // Handled in separate always block for W1C logic
                    end
                    ADDR_CS_REG: begin
                        reg_cs <= S_AXI_WDATA[0];
                    end
                    ADDR_START_REG: begin
                        reg_begin_exchange <= 1'b1;
                    end
                    ADDR_RESET_REG: begin
                        reg_soft_reset <= S_AXI_WDATA[0];
                    end
                    default: begin
                        // Read-only or invalid
                    end
                endcase
            end
        end
    end
    assign SPI_CS_N = reg_cs;
    //-------------------------------------------------------------------------
    // Read Channel Logic
    //-------------------------------------------------------------------------
    // State machine to handle FIFO read latency
    // State 0: Idle/Address Phase
    // State 1: Wait for FIFO (Latency)
    // State 2: Data Valid
    reg [1:0] r_state;
    localparam R_IDLE = 2'b00;
    localparam R_WAIT = 2'b01;
    localparam R_DONE = 2'b10;

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_arready  <= 1'b0;
            axi_rvalid   <= 1'b0;
            axi_rresp    <= 2'b0;
            axi_rdata    <= 32'b0;
            spi_rx_rd_en <= 1'b0;
            r_state      <= R_IDLE;
            r_reading_fifo <= 1'b0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    if (~axi_arready && S_AXI_ARVALID) begin
                        axi_arready <= 1'b1;
                        
                        // Address Decoding
                        case (S_AXI_ARADDR[C_S_AXI_ADDR_WIDTH-1:0])
                            ADDR_CTRL_REG: begin
                                axi_rdata <= {30'b0, reg_control};
                                r_state   <= R_DONE;
                            end
                            ADDR_STATUS_REG: begin
                                axi_rdata <= {29'b0, spi_busy, spi_rx_empty, spi_tx_full};
                                r_state   <= R_DONE;
                            end
                            ADDR_CLK_DIV_REG: begin
                                axi_rdata <= {24'b0, reg_clk_div};
                                r_state   <= R_DONE;
                            end
                            ADDR_RX_DATA_REG: begin
                                // Trigger FIFO Read
                                spi_rx_rd_en <= 1'b1;
                                r_reading_fifo <= 1'b1;
                                r_state      <= R_WAIT; // Go to wait state for FIFO latency
                            end
                            ADDR_IER_REG: begin
                                axi_rdata <= {30'b0, reg_ier};
                                r_state   <= R_DONE;
                            end
                            ADDR_ISR_REG: begin
                                axi_rdata <= {30'b0, isr_vector};
                                r_state   <= R_DONE;
                            end
                            ADDR_CS_REG: begin
                                axi_rdata <= {31'b0, reg_cs};
                                r_state   <= R_DONE;
                            end
                            ADDR_VERSION_REG: begin
                                axi_rdata <= C_IP_VERSION;
                                r_state   <= R_DONE;
                            end
                            default: begin
                                axi_rdata <= 32'b0;
                                r_state   <= R_DONE;
                            end
                        endcase
                    end
                end

                R_WAIT: begin
                    // Cycle 1: FIFO sees rd_en.
                    // We clear rd_en and wait for data to settle at output
                    axi_arready  <= 1'b0;
                    spi_rx_rd_en <= 1'b0;
                    r_state      <= R_DONE;
                    
                    // Note: In Sync_FIFO, data is valid 1 cycle after rd_en.
                    // We will latch it in the transition to R_DONE or output it there.
                    // Since axi_rdata is registered, we latch it here? 
                    // No, data is valid at the END of this cycle (start of next).
                    // So we latch it in the next state logic or use non-blocking here carefully.
                    // Actually, let's latch it in R_DONE entry.
                end

                R_DONE: begin
                    axi_arready <= 1'b0;
                    axi_rvalid  <= 1'b1;
                    
                    // If we came from R_WAIT (RX Data read), capture the FIFO output now
                    if (r_reading_fifo) begin 
                        axi_rdata <= {24'b0, spi_rx_byte};
                        r_reading_fifo <= 1'b0;
                    end

                    if (axi_rvalid && S_AXI_RREADY) begin
                        axi_rvalid <= 1'b0;
                        r_state    <= R_IDLE;
                    end
                end
            endcase
        end
    end

endmodule
