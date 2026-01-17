#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "xscugic.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "gpio_driver/gpio_driver.h"
#include "oled_driver/oled_driver.h"
#include "gpio_driver/btn_led_driver.h"
#include "i2c_axi_master/i2c_master_driver.h"
#include "spi_axi_master/spi_master_driver.h"
#include "hdc1080/hdc1080.h"
#include "timers.h"

typedef enum {
    MSG_TYPE_SENSOR,
    MSG_TYPE_TICK,
    MSG_TYPE_BUTTON
} MsgType_t;

typedef struct {
    MsgType_t type;
    union {
        struct { float temperature; float humidity; } sensor;
        uint32_t tick;
        uint32_t button_count;
    } payload;
} DisplayMessage_t;

/* Global instances */
static QueueHandle_t xDisplayQueue;
static BtnLed_Driver btn_led_inst;
static SPI_Master_Driver spi_inst;
static I2C_AXI_Master_Driver i2c_inst;
static GPIO_Driver gpio_inst;
static TimerHandle_t xButtonTimer = NULL;
static uint32_t PressCount = 0;
extern XScuGic xInterruptController;
volatile int transaction_complete;
volatile int transaction_error;
volatile int spi_transaction_complete;
static volatile int ButtonPressed = 0;

/*-----------------------------------------------------------*/
/* Button callback (safe to call FreeRTOS API) */
/* This is called from the ISR */
static void ButtonHandler(void *CallBackRef, u32 ButtonState)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 1. Disable Interrupts immediately to ignore subsequent bounces
    BtnLed_DisableInterrupts(&btn_led_inst);

    // 2. Start a timer to check for stable state (Debounce)
    if (xButtonTimer != NULL) {
        xTimerStartFromISR(xButtonTimer, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Timer Callback to handle button release and re-enable interrupts */
static void vButtonTimerCallback(TimerHandle_t xTimer)
{
    // Check if any button is still pressed
    u32 buttons = XGpio_DiscreteRead(&btn_led_inst.Gpio, 1);

    if (buttons & 0x0F) {
        // Button is pressed
        if (!ButtonPressed) {
            ButtonPressed = 1;
            PressCount++;
            DisplayMessage_t msg;
            msg.type = MSG_TYPE_BUTTON;
            msg.payload.button_count = PressCount;
            xQueueSend(xDisplayQueue, &msg, 0);
        }
        // Button is still held down. Restart timer to check again later.
        xTimerReset(xTimer, 0);
    } else {
        // Button released. Clear any latched interrupts from release bounce and re-enable.
        ButtonPressed = 0;
        btn_led_inst.LastButtons = 0;
        XGpio_InterruptClear(&btn_led_inst.Gpio, XGPIO_IR_CH1_MASK);
        BtnLed_EnableInterrupts(&btn_led_inst);
    }
}

/* Setup button interrupt safely */
static void SetupInterruptSystem(BtnLed_Driver *BtnLedInstancePtr)
{
    int Status;
    XScuGic_Config *IntcConfig;

    // 1. Lookup Config
    // We need this to ensure the driver struct is populated, even if we don't call CfgInitialize
    IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
    configASSERT(IntcConfig != NULL);

    // 2. CfgInitialize - SKIPPED
    // In FreeRTOS, the kernel initializes the GIC. Calling this again resets the hardware
    // and stops the Tick Timer, freezing the system.
    
    // WORKAROUND: Manually set IsReady and Config so driver API works
    xInterruptController.IsReady = XIL_COMPONENT_IS_READY;
    xInterruptController.Config = IntcConfig;

    // 3. Exception Handler - SKIPPED
    // FreeRTOS owns the exception table.

    // 4. Set Priority and Trigger Type
    // Priority: 0xA0 (Safe for FreeRTOS). Trigger: 0x1 (High Level).
    XScuGic_SetPriorityTriggerType(&xInterruptController, XPAR_FABRIC_AXI_GPIO_1_IP2INTC_IRPT_INTR, 0xA0, 0x1);

    // 5. Connect the ISR
    Status = XScuGic_Connect(&xInterruptController, XPAR_FABRIC_AXI_GPIO_1_IP2INTC_IRPT_INTR, 
                             (Xil_InterruptHandler)BtnLed_IntrHandler, BtnLedInstancePtr);
    configASSERT(Status == XST_SUCCESS);

    // 6. Enable the Interrupt
    XScuGic_Enable(&xInterruptController, XPAR_FABRIC_AXI_GPIO_1_IP2INTC_IRPT_INTR);
}

/*-----------------------------------------------------------*/
/* Tasks */
static void I2cTempMeasure_Task(void *pvParameters)
{
    DisplayMessage_t msg;

    xil_printf("I2C Task: Initializing Hardware...\r\n");
    I2C_AXI_Master_Init(&i2c_inst, XPAR_AXI_I2C_MASTER_0_BASEADDR, I2C_SCL_100KHZ, I2C_INTERRUPT_DISABLE);
    HDC1080_Init(&i2c_inst, 14, 14);

    for(;;) {
        msg.type = MSG_TYPE_SENSOR;
        HDC1080_GetTemperature(&i2c_inst, &msg.payload.sensor.temperature);
        HDC1080_GetHumidity(&i2c_inst, &msg.payload.sensor.humidity);

        xQueueSend(xDisplayQueue, &msg, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void TickGenerator_Task(void *pvParameters)
{
    // Initialize Interrupts here, AFTER the scheduler has started and initialized the GIC.
    SetupInterruptSystem(&btn_led_inst);
    BtnLed_EnableInterrupts(&btn_led_inst);

    DisplayMessage_t msg;
    for(;;) {
        msg.type = MSG_TYPE_TICK;
        msg.payload.tick = xTaskGetTickCount();

        // Heartbeat: Toggle LED 3 (Bit 3 of Channel 2)
        taskENTER_CRITICAL();
        btn_led_inst.LedState ^= 0x08;
        XGpio_DiscreteWrite(&btn_led_inst.Gpio, 2, btn_led_inst.LedState);
        taskEXIT_CRITICAL();

        xQueueSend(xDisplayQueue, &msg, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void SpiOledDisplay_Task(void *pvParameters)
{
    DisplayMessage_t recv_msg;
    char buffer[32];
    float last_temp = 0;
    float last_hum  = 0;
    uint32_t last_btn = 0;
    uint32_t last_tick = 0;

    // Initialize GPIO, SPI, OLED
    SPI_Master_Init(&spi_inst, XPAR_AXI_SPI_MASTER_0_BASEADDR, 0,0,40,SPI_INTERRUPT_DISABLE);
    OLED_Init(&spi_inst, &gpio_inst);

    for(;;)
    {
        if(xQueueReceive(xDisplayQueue, &recv_msg, portMAX_DELAY) == pdPASS)
        {
            switch(recv_msg.type) {
                case MSG_TYPE_SENSOR:
                    last_temp = recv_msg.payload.sensor.temperature;
                    last_hum  = recv_msg.payload.sensor.humidity;
                    snprintf(buffer,sizeof(buffer),"Temp:%d.%02dC",(int)last_temp,(int)((last_temp-(int)last_temp)*100));
                    OLED_SetCursor(0,0); OLED_PutString8x8(buffer);
                    snprintf(buffer,sizeof(buffer),"Hum:%d.%02d%%",(int)last_hum,(int)((last_hum-(int)last_hum)*100));
                    OLED_SetCursor(1,0); OLED_PutString8x8(buffer);
                    break;

                case MSG_TYPE_TICK:
                    last_tick = recv_msg.payload.tick;
                    snprintf(buffer,sizeof(buffer),"Time:%lu s",last_tick/configTICK_RATE_HZ);
                    OLED_SetCursor(3,0); OLED_PutString8x8(buffer);
                    snprintf(buffer,sizeof(buffer),"Btn:%lu",last_btn);
                    OLED_SetCursor(2,0); OLED_PutString8x8(buffer);
                    break;

                case MSG_TYPE_BUTTON:
                    last_btn = recv_msg.payload.button_count;

                    // Flash Message
                    OLED_Clear();
                    OLED_SetCursor(3, 0);
                    OLED_PutString8x8("BUTTON PRESSED!");

                    vTaskDelay(pdMS_TO_TICKS(600));

                    // Restore Screen
                    OLED_Clear();
                    snprintf(buffer,sizeof(buffer),"Temp:%d.%02dC",(int)last_temp,(int)((last_temp-(int)last_temp)*100));
                    OLED_SetCursor(0,0); OLED_PutString8x8(buffer);
                    snprintf(buffer,sizeof(buffer),"Hum:%d.%02d%%",(int)last_hum,(int)((last_hum-(int)last_hum)*100));
                    OLED_SetCursor(1,0); OLED_PutString8x8(buffer);
                    snprintf(buffer,sizeof(buffer),"Time:%lu s",last_tick/configTICK_RATE_HZ);
                    OLED_SetCursor(3,0); OLED_PutString8x8(buffer);
                    snprintf(buffer,sizeof(buffer),"Btn:%lu",last_btn);
                    OLED_SetCursor(2,0); OLED_PutString8x8(buffer);
                    break;
            }
        }
    }
}

extern XScuGic xInterruptController;

int main()
{
    XScuGic_Config *scuGicConfig = XScuGic_LookupConfig(0);
    XScuGic_CfgInitialize(&xInterruptController, scuGicConfig, scuGicConfig->CpuBaseAddress);
    xil_printf("--- FreeRTOS Zynq Demo ---\r\n");

    xDisplayQueue = xQueueCreate(10,sizeof(DisplayMessage_t));
    configASSERT(xDisplayQueue);

    // Create Debounce Timer (One-shot, 50ms initial delay)
    xButtonTimer = xTimerCreate("BtnTmr", pdMS_TO_TICKS(50), pdFALSE, (void *)0, vButtonTimerCallback);

    // Initialize button driver
    BtnLed_Init(&btn_led_inst, XPAR_AXI_GPIO_1_DEVICE_ID);
    BtnLed_SetHandler(&btn_led_inst, ButtonHandler, NULL);

    // Initialize OLED GPIOs
    GPIO_Init(&gpio_inst, XPAR_AXI_GPIO_0_DEVICE_ID);
    GPIO_SetPin(&gpio_inst, GPIO_PIN_VDDC|GPIO_PIN_VBATC, 1);

    // Setup interrupts safely
    // Create tasks
    xTaskCreate(I2cTempMeasure_Task,"Temp",configMINIMAL_STACK_SIZE*4,NULL,2,NULL);
    xTaskCreate(TickGenerator_Task,"Tick",configMINIMAL_STACK_SIZE*2,NULL,2,NULL);
    xTaskCreate(SpiOledDisplay_Task,"OLED",configMINIMAL_STACK_SIZE*6,NULL,1,NULL);

    vTaskStartScheduler();

    for(;;);
}
