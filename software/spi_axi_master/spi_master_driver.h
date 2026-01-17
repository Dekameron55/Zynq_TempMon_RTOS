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

#ifndef SPI_MASTER_DRIVER_H
#define SPI_MASTER_DRIVER_H

#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"

/************************** Constant Definitions *****************************/

// Register Map
typedef enum {
    SPI_REG_CTRL       = 0x00,
    SPI_REG_STATUS     = 0x04,
    SPI_REG_CLK_DIV    = 0x08,
    SPI_REG_TX_DATA    = 0x0C,
    SPI_REG_RX_DATA    = 0x10,
    SPI_REG_IER        = 0x14,
    SPI_REG_ISR        = 0x18,
    SPI_REG_CS         = 0x1C,
    SPI_REG_VERSION    = 0x20,
    SPI_REG_START_ADDR = 0x24,
	SPI_REG_RESET_ADDR = 0x28
} SPI_RegisterMap;

// Control Register Bits
typedef enum {
    SPI_CTRL_BIT_CPOL   = (1 << 0),
    SPI_CTRL_BIT_CPHA   = (1 << 1),
} SPI_ControlBits;

// Status Register Bits
typedef enum {
    SPI_STATUS_BIT_TX_FULL  = (1 << 0),
    SPI_STATUS_BIT_RX_EMPTY = (1 << 1),
    SPI_STATUS_BIT_BUSY     = (1 << 2)
} SPI_StatusBits;

// Interrupt Bits (IER & ISR)
typedef enum {
    SPI_INTR_BIT_TX_HALF_EMPTY = (1 << 0),
    SPI_INTR_BIT_EXCHANGE_END  = (1 << 1)
} SPI_InterruptBits;

#define SPI_AXI_TIMEOUT_DEFAULT 1000 // Default timeout in milliseconds

#define SPI_IP_VERSION_EXPECTED 0x00010300

/**************************** Type Definitions *******************************/

typedef enum {
    SPI_AXI_SUCCESS = XST_SUCCESS,
    SPI_AXI_FAILURE = XST_FAILURE,
    SPI_AXI_DEVICE_BUSY = XST_DEVICE_BUSY,
    SPI_AXI_INVALID_PARAM = XST_INVALID_PARAM,
    SPI_AXI_TIMEOUT
} SPI_Status;

/**
 * Chip Select State
 */
typedef enum {
    SPI_CS_ACTIVE_LOW    = 0x00,
    SPI_CS_INACTIVE_HIGH = 0x01
} SPI_ChipSelectState;

/**
 * Enum to specify whether to enable interrupts in the core during Init.
 */
typedef enum {
    SPI_INTERRUPT_DISABLE,
    SPI_INTERRUPT_ENABLE
} SPI_InterruptEnable;

/**
 * Callback function type for handling asynchronous events.
 */
typedef void (*SPI_Handler)(void *CallBackRef, u32 Event, u32 ByteCount);

/**
 * Driver instance struct.
 */
typedef struct {
    UINTPTR BaseAddress;
    u32 IsReady;
    
    // Interrupt / Async State
    SPI_Handler Handler;
    void *CallBackRef;
    u32 RequestedByteCount;
    u8 *RecvBuffer;         // Pointer to user buffer for RX data
} SPI_Master_Driver;

/***************** Macros (Inline Functions) Definitions *********************/

#define SPI_Master_In32(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))

#define SPI_Master_Out32(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (Data))

/************************** Function Prototypes ******************************/

/**
 * Initialize the SPI Master driver with all configuration parameters.
 * @param InstancePtr Pointer to the driver instance.
 * @param BaseAddress Physical base address of the IP.
 * @param CPOL Clock Polarity (0 or 1).
 * @param CPHA Clock Phase (0 or 1).
 * @param ClkDiv Clock Divider value.
 * @param EnableInterrupts Enable or Disable interrupts at init.
 * @return SPI_AXI_SUCCESS or SPI_AXI_FAILURE.
 */
SPI_Status SPI_Master_Init(SPI_Master_Driver *InstancePtr, UINTPTR BaseAddress, u8 CPOL, u8 CPHA, u8 ClkDiv, SPI_InterruptEnable EnableInterrupts);

/**
 * Manually control the Chip Select line.
 * @param Assert If 1, CS goes Low (Active). If 0, CS goes High (Inactive).
 */
void SPI_Master_SetCS(SPI_Master_Driver *InstancePtr, u8 Assert);

/**
 * Blocking transfer of data.
 * @param SendBuf Pointer to data to send (can be NULL).
 * @param RecvBuf Pointer to buffer for received data (can be NULL).
 * @param ByteCount Number of bytes to transfer.
 */
SPI_Status SPI_Master_Transfer(SPI_Master_Driver *InstancePtr, u8 *SendBuf, u8 *RecvBuf, u32 ByteCount, u32 TimeoutMs);

/**
 * Non-blocking (Interrupt-driven) transfer.
 * Starts the transfer and returns immediately.
 * @param SendBuf Pointer to data to send (can be NULL).
 * @param RecvBuf Pointer to buffer for received data (can be NULL).
 * @param ByteCount Number of bytes to transfer.
 */
SPI_Status SPI_Master_Transfer_IT(SPI_Master_Driver *InstancePtr, u8 *SendBuf, u8 *RecvBuf, u32 ByteCount);

/**
 * Sets the callback function for handling asynchronous events.
 */
void SPI_Master_SetHandler(SPI_Master_Driver *InstancePtr, SPI_Handler FuncPtr, void *CallBackRef);

/**
 * The Interrupt Service Routine (ISR).
 */
void SPI_Master_InterruptHandler(void *InstancePtr);

#endif // SPI_MASTER_DRIVER_H
