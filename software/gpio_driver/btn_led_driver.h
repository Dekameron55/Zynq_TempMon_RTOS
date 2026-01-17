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

#ifndef BTN_LED_DRIVER_H
#define BTN_LED_DRIVER_H

#include "xil_types.h"
#include "xgpio.h"

typedef void (*BtnLed_Handler)(void *CallBackRef, u32 ButtonState);

typedef struct {
    XGpio Gpio;
    u32 LedState; // Shadow register to keep track of LED state for toggling
    u32 LastButtons; // Track previous state for edge detection
    BtnLed_Handler Handler;
    void *CallBackRef;
} BtnLed_Driver;

int BtnLed_Init(BtnLed_Driver *InstancePtr, u16 DeviceId);
void BtnLed_SetHandler(BtnLed_Driver *InstancePtr, BtnLed_Handler FuncPtr, void *CallBackRef);
void BtnLed_EnableInterrupts(BtnLed_Driver *InstancePtr);
void BtnLed_DisableInterrupts(BtnLed_Driver *InstancePtr);
void BtnLed_IntrHandler(void *InstancePtr);

#endif /* BTN_LED_DRIVER_H */
