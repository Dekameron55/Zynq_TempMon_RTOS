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

#include "btn_led_driver.h"

// Channel Definitions based on your Block Design
#define CHANNEL_BUTTONS 1 // Channel 1: 4 Bits Input
#define CHANNEL_LEDS    2 // Channel 2: 4 Bits Output

int BtnLed_Init(BtnLed_Driver *InstancePtr, u16 DeviceId) {
    int Status;

    // Initialize the XGpio instance
    Status = XGpio_Initialize(&InstancePtr->Gpio, DeviceId);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Set Data Direction
    // Channel 1 (Buttons): All Inputs (0xFFFFFFFF)
    // Channel 2 (LEDs): All Outputs (0x00000000)
    XGpio_SetDataDirection(&InstancePtr->Gpio, CHANNEL_BUTTONS, 0xFFFFFFFF);
    XGpio_SetDataDirection(&InstancePtr->Gpio, CHANNEL_LEDS,    0x00000000);

    // Initialize LEDs to OFF (0)
    InstancePtr->LedState = 0;
    XGpio_DiscreteWrite(&InstancePtr->Gpio, CHANNEL_LEDS, InstancePtr->LedState);

    InstancePtr->LastButtons = 0;
    InstancePtr->Handler = NULL;
    InstancePtr->CallBackRef = NULL;

    return XST_SUCCESS;
}

void BtnLed_SetHandler(BtnLed_Driver *InstancePtr, BtnLed_Handler FuncPtr, void *CallBackRef) {
    InstancePtr->Handler = FuncPtr;
    InstancePtr->CallBackRef = CallBackRef;
}

void BtnLed_EnableInterrupts(BtnLed_Driver *InstancePtr) {
    XGpio_InterruptEnable(&InstancePtr->Gpio, XGPIO_IR_CH1_MASK);
    XGpio_InterruptGlobalEnable(&InstancePtr->Gpio);
}

void BtnLed_DisableInterrupts(BtnLed_Driver *InstancePtr) {
    XGpio_InterruptDisable(&InstancePtr->Gpio, XGPIO_IR_CH1_MASK);
}

void BtnLed_IntrHandler(void *InstancePtr) {
    BtnLed_Driver *Driver = (BtnLed_Driver *)InstancePtr;
    u32 Buttons;
    u32 InterruptStatus;

    // Read Interrupt Status
    InterruptStatus = XGpio_InterruptGetStatus(&Driver->Gpio);

    // Check if Channel 1 (Buttons) caused the interrupt
    if (InterruptStatus & XGPIO_IR_CH1_MASK) {
        
        // Read current button state
        Buttons = XGpio_DiscreteRead(&Driver->Gpio, CHANNEL_BUTTONS);

        // Detect Rising Edges (Presses): (Current is 1 AND Last was 0)
        u32 RisingEdges = Buttons & ~Driver->LastButtons;

        if (RisingEdges) {
            // Toggle corresponding LEDs based on which button was pressed
            // We only care about the bottom 4 bits
            u32 ToggleMask = RisingEdges & 0x0F;
            
            Driver->LedState ^= ToggleMask;
            XGpio_DiscreteWrite(&Driver->Gpio, CHANNEL_LEDS, Driver->LedState);

            // Call the user handler if registered
            if (Driver->Handler) {
                Driver->Handler(Driver->CallBackRef, RisingEdges);
            }
        }

        // Update Last State
        Driver->LastButtons = Buttons;

        // Clear the Interrupt
        XGpio_InterruptClear(&Driver->Gpio, XGPIO_IR_CH1_MASK);
    }
}
