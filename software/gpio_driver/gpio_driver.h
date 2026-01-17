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

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include "xil_types.h"
#include "xgpio.h"

// Pin Mapping (Sequentially 0:3)
// 7  D/C   -> Bit 0
// 8  RES   -> Bit 1
// 9  VBATC -> Bit 2
// 10 VDDC  -> Bit 3
typedef enum {
    GPIO_PIN_DC    = (1 << 0), // Data/Command
    GPIO_PIN_RES   = (1 << 1), // Reset
    GPIO_PIN_VBATC = (1 << 2), // Vbat Control
    GPIO_PIN_VDDC  = (1 << 3)  // Vdd Control
} GPIO_Pin;

typedef struct {
    XGpio Gpio;
} GPIO_Driver;

/************************** Function Prototypes ******************************/

/**
 * Initialize the GPIO driver.
 * Configures the specified pins as outputs.
 * @param InstancePtr Pointer to the driver instance.
 * @param DeviceId Device ID of the AXI GPIO IP.
 */
void GPIO_Init(GPIO_Driver *InstancePtr, u16 DeviceId);

/**
 * Set the state of a specific pin or mask of pins.
 * @param InstancePtr Pointer to the driver instance.
 * @param PinMask Bitmask of pins to modify (use GPIO_Pin enum).
 * @param Value 1 to set High, 0 to set Low.
 */
void GPIO_SetPin(GPIO_Driver *InstancePtr, u32 PinMask, u8 Value);

/**
 * Toggle a specific pin or mask of pins.
 * @param InstancePtr Pointer to the driver instance.
 * @param PinMask Bitmask of pins to toggle.
 */
void GPIO_TogglePin(GPIO_Driver *InstancePtr, u32 PinMask);

/**
 * Get the current state of the output register (Shadow).
 * @param InstancePtr Pointer to the driver instance.
 * @return Current value of the output register.
 */
u32 GPIO_GetState(GPIO_Driver *InstancePtr);

#endif /* GPIO_DRIVER_H_ */
