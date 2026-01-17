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

#include "spi_master_driver.h"
#include "sleep.h" // For usleep function

static int SPI_Master_IsBusy(SPI_Master_Driver *InstancePtr);

/*****************************************************************************/
/**
* Initializes the SPI Master driver instance with all parameters.
*
* @param    InstancePtr is a pointer to the SPI_Master_Driver instance.
* @param    BaseAddress is the physical base address of the IP core.
* @param    CPOL Clock Polarity (0 or 1).
* @param    CPHA Clock Phase (0 or 1).
* @param    ClkDiv Clock Divider.
* @param    EnableInterrupts SPI_INTERRUPT_ENABLE or SPI_INTERRUPT_DISABLE.
*
* @return   SPI_AXI_SUCCESS if successful, SPI_AXI_FAILURE if version mismatch.
*
******************************************************************************/
SPI_Status SPI_Master_Init(SPI_Master_Driver *InstancePtr, UINTPTR BaseAddress, u8 CPOL, u8 CPHA, u8 ClkDiv, SPI_InterruptEnable EnableInterrupts)
{
    u32 Version;
    u32 CtrlReg = 0;

    Xil_AssertNonvoid(InstancePtr != NULL);

    InstancePtr->BaseAddress = BaseAddress;
    InstancePtr->IsReady = 0;
    InstancePtr->Handler = NULL;
    InstancePtr->CallBackRef = NULL;
    InstancePtr->RecvBuffer = NULL;
    InstancePtr->RequestedByteCount = 0;

    // 1. Verify IP Version
    Version = SPI_Master_In32(BaseAddress, SPI_REG_VERSION);
    if (Version != SPI_IP_VERSION_EXPECTED) {
        return SPI_AXI_FAILURE;
    }

    // 2. Configure Control Register (CPOL, CPHA)
    if (CPOL) CtrlReg |= SPI_CTRL_BIT_CPOL;
    if (CPHA) CtrlReg |= SPI_CTRL_BIT_CPHA;
    SPI_Master_Out32(BaseAddress, SPI_REG_CTRL, CtrlReg);

    // 3. Set Clock Divider
    SPI_Master_Out32(BaseAddress, SPI_REG_CLK_DIV, (u32)ClkDiv);

    // 4. Set Default State: CS Inactive
    SPI_Master_Out32(BaseAddress, SPI_REG_CS, SPI_CS_INACTIVE_HIGH);

    // 5. Clear any pending interrupts
    SPI_Master_Out32(BaseAddress, SPI_REG_ISR, 0xFFFFFFFF);

    // 6. Configure Interrupts
    if (EnableInterrupts == SPI_INTERRUPT_ENABLE) {
        // We enable Exchange End by default if interrupts are requested, 
        // but usually we only enable them during a transfer. 
        // For now, we leave IER 0 until a transfer starts.
        SPI_Master_Out32(BaseAddress, SPI_REG_IER, 0x00);
    } else {
        SPI_Master_Out32(BaseAddress, SPI_REG_IER, 0x00);
    }

    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return SPI_AXI_SUCCESS;
}

/*****************************************************************************/
/**
* Controls the Chip Select (CS) line.
*
* @param    Assert: 1 to Assert (Low), 0 to Deassert (High)
*
******************************************************************************/
void SPI_Master_SetCS(SPI_Master_Driver *InstancePtr, u8 Assert)
{
    if (Assert) {
        SPI_Master_Out32(InstancePtr->BaseAddress, SPI_REG_CS, SPI_CS_ACTIVE_LOW);
    } else {
        SPI_Master_Out32(InstancePtr->BaseAddress, SPI_REG_CS, SPI_CS_INACTIVE_HIGH);
    }
}

/*****************************************************************************/
/**
* Checks if the SPI core is currently busy with a transaction.
*
******************************************************************************/
static int SPI_Master_IsBusy(SPI_Master_Driver *InstancePtr)
{
	// The core is busy if the BUSY bit in the status register is set.
	return (SPI_Master_In32(InstancePtr->BaseAddress, SPI_REG_STATUS) & SPI_STATUS_BIT_BUSY);
}

/*****************************************************************************/
/**
* Performs a blocking transfer.
*
* @param    SendBuf: Buffer containing data to send.
* @param    RecvBuf: Buffer to store received data.
* @param    ByteCount: Number of bytes to transfer.
* @param    TimeoutMs: Timeout duration in milliseconds.
*
******************************************************************************/
SPI_Status SPI_Master_Transfer(SPI_Master_Driver *InstancePtr, u8 *SendBuf, u8 *RecvBuf, u32 ByteCount, u32 TimeoutMs)
{
    u32 BytesSent = 0;
    u8 TxData;
    u32 TimeoutCounter = 0;

    if (InstancePtr->IsReady != XIL_COMPONENT_IS_READY) return SPI_AXI_FAILURE;
    if (ByteCount == 0) return SPI_AXI_FAILURE;
    // NOTE: This function assumes ByteCount will fit into the hardware FIFO (1KB).

    // Check if device is busy
    if (SPI_Master_IsBusy(InstancePtr)) {
        return SPI_AXI_DEVICE_BUSY;
    }

    // Clear any stale 'Exchange End' interrupts before starting
    SPI_Master_Out32(InstancePtr->BaseAddress, SPI_REG_ISR, SPI_INTR_BIT_EXCHANGE_END);

    // 1. Fill the TX FIFO with all data for the transaction.
    while (BytesSent < ByteCount) {
        if (SPI_Master_In32(InstancePtr->BaseAddress, SPI_REG_STATUS) & SPI_STATUS_BIT_TX_FULL) {
            // This should not happen if ByteCount <= FIFO depth.
            return SPI_AXI_FAILURE;
        }
        TxData = (SendBuf != NULL) ? SendBuf[BytesSent] : 0x00;
        SPI_Master_Out32(InstancePtr->BaseAddress, SPI_REG_TX_DATA, TxData);
        BytesSent++;
    }

    // 2. Trigger the exchange.
    SPI_Master_Out32(InstancePtr->BaseAddress, SPI_REG_START_ADDR, 1);

    // 3. Wait for the transaction to complete by polling the 'Exchange End' flag in the ISR.
    while ((SPI_Master_In32(InstancePtr->BaseAddress, SPI_REG_ISR) & SPI_INTR_BIT_EXCHANGE_END) == 0) {
        // Check for timeout
        if (TimeoutMs > 0) {
            usleep(1); // Poll every microsecond
            TimeoutCounter++;
            if (TimeoutCounter > (TimeoutMs * 1000)) {
                return SPI_AXI_TIMEOUT;
            }
        }
    }

    // 4. Clear the 'Exchange End' interrupt flag (W1C).
    SPI_Master_Out32(InstancePtr->BaseAddress, SPI_REG_ISR, SPI_INTR_BIT_EXCHANGE_END);

    // 5. Read the received data from the RX FIFO.
    if (RecvBuf != NULL) {
        for (u32 i = 0; i < ByteCount; i++) {
            RecvBuf[i] = (u8)SPI_Master_In32(InstancePtr->BaseAddress, SPI_REG_RX_DATA);
        }
    } else {
        // If user doesn't want the data, we still need to flush the FIFO.
        for (u32 i = 0; i < ByteCount; i++) {
            (void)SPI_Master_In32(InstancePtr->BaseAddress, SPI_REG_RX_DATA);
        }
    }
    return SPI_AXI_SUCCESS;
}

/*****************************************************************************/
/**
* Performs a non-blocking (Interrupt-Driven) transfer.
* This function fills the FIFO and enables the interrupt, then returns.
*
* @param    SendBuf: Buffer containing data to send.
* @param    RecvBuf: Buffer to store received data.
* @param    ByteCount: Number of bytes to transfer.
*
******************************************************************************/
SPI_Status SPI_Master_Transfer_IT(SPI_Master_Driver *InstancePtr, u8 *SendBuf, u8 *RecvBuf, u32 ByteCount)
{
    u32 BytesSent = 0;
    u8 TxData;

    if (InstancePtr->IsReady != XIL_COMPONENT_IS_READY) return SPI_AXI_FAILURE;
    if (ByteCount == 0) return SPI_AXI_FAILURE;

    // Check if device is busy
    if (SPI_Master_IsBusy(InstancePtr)) {
        return SPI_AXI_DEVICE_BUSY;
    }

    // Clear any stale 'Exchange End' interrupts before starting
    SPI_Master_Out32(InstancePtr->BaseAddress, SPI_REG_ISR, SPI_INTR_BIT_EXCHANGE_END);

    // Store state for ISR
    InstancePtr->RecvBuffer = RecvBuf;
    InstancePtr->RequestedByteCount = ByteCount;

    // Fill TX FIFO
    // Note: This simple implementation assumes ByteCount fits in the FIFO (1KB).
    // If ByteCount > 1KB, we would need to use the TX_HALF_EMPTY interrupt to refill.
    while ((BytesSent < ByteCount) && !(SPI_Master_In32(InstancePtr->BaseAddress, SPI_REG_STATUS) & SPI_STATUS_BIT_TX_FULL)) {
        TxData = (SendBuf != NULL) ? SendBuf[BytesSent] : 0x00;
        SPI_Master_Out32(InstancePtr->BaseAddress, SPI_REG_TX_DATA, TxData);
        BytesSent++;
    }

    if (BytesSent < ByteCount) {
        // FIFO Full and we have more data. 
        // In a robust driver, we would enable TX_HALF_EMPTY here.
        return SPI_AXI_DEVICE_BUSY; 
    }

    // Enable "Exchange End" Interrupt
    SPI_Master_Out32(InstancePtr->BaseAddress, SPI_REG_IER, SPI_INTR_BIT_EXCHANGE_END);

    // Trigger Exchange
    SPI_Master_Out32(InstancePtr->BaseAddress, SPI_REG_START_ADDR, 1);


    return SPI_AXI_SUCCESS;
}

/*****************************************************************************/
/**
* Sets the callback function.
*
******************************************************************************/
void SPI_Master_SetHandler(SPI_Master_Driver *InstancePtr, SPI_Handler FuncPtr, void *CallBackRef)
{
    InstancePtr->Handler = FuncPtr;
    InstancePtr->CallBackRef = CallBackRef;
}

/*****************************************************************************/
/**
* Interrupt Service Routine.
*
******************************************************************************/
void SPI_Master_InterruptHandler(void *InstancePtr)
{
    SPI_Master_Driver *SpiInst = (SPI_Master_Driver *)InstancePtr;
    u32 IsrStatus = SPI_Master_In32(SpiInst->BaseAddress, SPI_REG_ISR);

    if (IsrStatus & SPI_INTR_BIT_EXCHANGE_END) {
        // Transaction Complete. Read RX FIFO.
        for (u32 i = 0; i < SpiInst->RequestedByteCount; i++) {
            u32 RxData = SPI_Master_In32(SpiInst->BaseAddress, SPI_REG_RX_DATA);
            if (SpiInst->RecvBuffer != NULL) {
                SpiInst->RecvBuffer[i] = (u8)RxData;
            }
        }

        // Disable Interrupts until next transfer
        SPI_Master_Out32(SpiInst->BaseAddress, SPI_REG_IER, 0x00);

        // Call User Callback
        if (SpiInst->Handler) {
            SpiInst->Handler(SpiInst->CallBackRef, SPI_INTR_BIT_EXCHANGE_END, SpiInst->RequestedByteCount);
        }
    }

    // Clear Interrupts (Write 1 to Clear)
    SPI_Master_Out32(SpiInst->BaseAddress, SPI_REG_ISR, IsrStatus);
}
