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

#include "gpio_driver.h"

#define GPIO_CHANNEL 1

void GPIO_Init(GPIO_Driver *InstancePtr, u16 DeviceId)
{
    int Status;

    // 1. Initialize the XGpio instance
    Status = XGpio_Initialize(&InstancePtr->Gpio, DeviceId);
    if (Status != XST_SUCCESS) {
        return;
    }

    // 2. Set Pin Direction
    // Mask: 0 = Output, 1 = Input.
    // We set our specific pins to Output (0) and leave others as Input (1) for safety.
    u32 DirectionMask = ~(GPIO_PIN_DC | GPIO_PIN_RES | GPIO_PIN_VBATC | GPIO_PIN_VDDC);
    XGpio_SetDataDirection(&InstancePtr->Gpio, GPIO_CHANNEL, DirectionMask);

    // 3. Initialize Outputs to Safe State (High = OFF for Active Low Power/Reset)
    XGpio_DiscreteSet(&InstancePtr->Gpio, GPIO_CHANNEL, (GPIO_PIN_DC | GPIO_PIN_RES | GPIO_PIN_VBATC | GPIO_PIN_VDDC));
}

void GPIO_SetPin(GPIO_Driver *InstancePtr, u32 PinMask, u8 Value)
{
    if (Value) {
        // Set bits High
        XGpio_DiscreteSet(&InstancePtr->Gpio, GPIO_CHANNEL, PinMask);
    } else {
        // Set bits Low
        XGpio_DiscreteClear(&InstancePtr->Gpio, GPIO_CHANNEL, PinMask);
    }
}

void GPIO_TogglePin(GPIO_Driver *InstancePtr, u32 PinMask)
{
    u32 Current = XGpio_DiscreteRead(&InstancePtr->Gpio, GPIO_CHANNEL);
    XGpio_DiscreteWrite(&InstancePtr->Gpio, GPIO_CHANNEL, Current ^ PinMask);
}

u32 GPIO_GetState(GPIO_Driver *InstancePtr)
{
    return XGpio_DiscreteRead(&InstancePtr->Gpio, GPIO_CHANNEL);
}
