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

#include "oled_driver.h"
#include "sleep.h"
#include "oled_font.h"

// SSD1306 Commands
#define OLED_CMD_DISPLAY_OFF        0xAE
#define OLED_CMD_SET_CHARGE_PUMP    0x8D
#define OLED_CMD_DISPLAY_ON         0xAF
#define OLED_CMD_SET_SEG_REMAP      0xA1
#define OLED_CMD_SET_COM_DIR        0xC8
#define OLED_CMD_SET_COM_PINS       0xDA
#define OLED_CMD_SET_CONTRAST       0x81
#define OLED_CMD_SET_PRECHARGE      0xD9
#define OLED_CMD_SET_VCOM_DETECT    0xDB
#define OLED_CMD_ENTIRE_DISPLAY_ON  0xA5
#define OLED_CMD_NORMAL_DISPLAY     0xA6
#define OLED_CMD_DISPLAY_RAM        0xA4

static SPI_Master_Driver *OledSpi;
static GPIO_Driver *OledGpio;

static SPI_Status OLED_WriteByte(u8 Data, u8 IsData)
{
    u8 SendBuf[1];
    SPI_Status Status;
    SendBuf[0] = Data;

    // Set Data/Command Pin
    // 0 = Command, 1 = Data
    if (IsData) {
        GPIO_SetPin(OledGpio, GPIO_PIN_DC, 1);
    } else {
        GPIO_SetPin(OledGpio, GPIO_PIN_DC, 0);
    }

    // Assert Chip Select (Active Low)
    SPI_Master_SetCS(OledSpi, 1);

    // Send via SPI
    Status = SPI_Master_Transfer(OledSpi, SendBuf, NULL, 1, 100);

    // Deassert Chip Select
    SPI_Master_SetCS(OledSpi, 0);

    return Status;
}

static SPI_Status OLED_WriteCommand(u8 Cmd)
{
    return OLED_WriteByte(Cmd, 0);
}

SPI_Status OLED_Init(SPI_Master_Driver *SpiInst, GPIO_Driver *GpioInst)
{
    OledSpi = SpiInst;
    OledGpio = GpioInst;
    SPI_Status Status;

    // 1. Power-On Sequence
    // Ensure everything is off initially
    // PmodOLED Power Control is Active Low (1 = OFF, 0 = ON). Reset is Active Low.
    GPIO_SetPin(OledGpio, GPIO_PIN_VDDC, 1);   // Logic Power Off
    GPIO_SetPin(OledGpio, GPIO_PIN_VBATC, 1);  // OLED Power Off
    GPIO_SetPin(OledGpio, GPIO_PIN_RES, 1);    // Reset Inactive (High)
    usleep(1000); 

    // Turn on Logic Power (VDD)
    GPIO_SetPin(OledGpio, GPIO_PIN_VDDC, 0);  // Logic Power ON
    usleep(1000); // Wait 1ms for logic to stabilize

    // 2. Send Display Off Command
    Status = OLED_WriteCommand(OLED_CMD_DISPLAY_OFF);
    if (Status != SPI_AXI_SUCCESS) return Status;

    // 3. Toggle Reset
    GPIO_SetPin(OledGpio, GPIO_PIN_RES, 0);   // Reset Low
    usleep(1000);
    GPIO_SetPin(OledGpio, GPIO_PIN_RES, 1);   // Reset High
    usleep(1000);

    // 4. Charge Pump and Precharge
    OLED_WriteCommand(OLED_CMD_SET_CHARGE_PUMP);
    OLED_WriteCommand(0x14); // Enable Charge Pump

    OLED_WriteCommand(OLED_CMD_SET_PRECHARGE);
    OLED_WriteCommand(0xF1);

    // 5. Turn on OLED Power (VCC/VBAT)
    GPIO_SetPin(OledGpio, GPIO_PIN_VBATC, 0); // OLED Power ON
    usleep(100000); // Wait 100ms for high voltage to stabilize

    // 6. Configuration Commands
    OLED_WriteCommand(OLED_CMD_SET_SEG_REMAP);
    OLED_WriteCommand(OLED_CMD_SET_COM_DIR);

    OLED_WriteCommand(OLED_CMD_SET_COM_PINS);
    OLED_WriteCommand(0x20); // Sequential COM, Left/Right remap enabled (Matches reference)

    // 7. Turn on Display
    OLED_WriteCommand(OLED_CMD_DISPLAY_ON);

    // Set to Normal Display (RAM content)
    OLED_WriteCommand(OLED_CMD_DISPLAY_RAM); // 0xA4: Resume to RAM content
    OLED_WriteCommand(OLED_CMD_NORMAL_DISPLAY); // 0xA6: Normal (not inverted)
    
    // Clear the screen to remove random noise
    OLED_Clear();

    return SPI_AXI_SUCCESS;
}

SPI_Status OLED_SetCursor(u8 Page, u8 Column)
{
    if (Page > 7) Page = 7;
    if (Column > 127) Column = 127;

    // Set Page Address
    OLED_WriteCommand(0xB0 + Page);
    
    // Set Column Address (Lower and Higher Nibbles)
    OLED_WriteCommand(0x00 | (Column & 0x0F));
    OLED_WriteCommand(0x10 | ((Column >> 4) & 0x0F));

    return SPI_AXI_SUCCESS;
}

SPI_Status OLED_Clear(void)
{
    u8 page, col;
    for (page = 0; page < 8; page++) {
        OLED_SetCursor(page, 0);
        for (col = 0; col < 128; col++) {
            OLED_WriteByte(0x00, 1); // Write Data 0x00
        }
    }
    return SPI_AXI_SUCCESS;
}

SPI_Status OLED_PutChar8x8(char c)
{
    if (c < 32 || c > 126) return SPI_AXI_FAILURE;

    u8 i;
    u8 index = c - 32;

    // Write 8 bytes of font data
    for (i = 0; i < 8; i++) {
        OLED_WriteByte(OLED_Font8x8[index][i], 1);
    }
    // No extra spacing needed for 8x8 as it fills the block

    return SPI_AXI_SUCCESS;
}

SPI_Status OLED_PutString8x8(const char *str)
{
    while (*str) {
        OLED_PutChar8x8(*str);
        str++;
    }
    return SPI_AXI_SUCCESS;
}

SPI_Status OLED_PutChar(char c)
{
    if (c < 32 || c > 126) return SPI_AXI_FAILURE;

    u8 i;
    u8 index = c - 32; // Font starts at space (32)

    // Write 5 bytes of font data
    for (i = 0; i < 5; i++) {
        OLED_WriteByte(OLED_Font5x7[index][i], 1);
    }

    // Write 1 byte of spacing (0x00)
    OLED_WriteByte(0x00, 1);

    return SPI_AXI_SUCCESS;
}

SPI_Status OLED_PutString(const char *str)
{
    while (*str) {
        OLED_PutChar(*str);
        str++;
    }
    return SPI_AXI_SUCCESS;
}
