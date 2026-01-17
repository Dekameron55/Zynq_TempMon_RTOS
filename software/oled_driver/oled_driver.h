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

#ifndef OLED_DRIVER_H_
#define OLED_DRIVER_H_

#include "xil_types.h"
#include "../spi_axi_master/spi_master_driver.h"
#include "../gpio_driver/gpio_driver.h"

// OLED Dimensions
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// OLED Memory Organization (SSD1306 Page Addressing Mode):
// The 128x64 display is divided into 8 "Pages" (Page 0 - Page 7).
// Each Page handles a horizontal strip of the screen that is 8 pixels tall.
//
// +---------------------------------------------------------------+
// | Page 0 (Rows 0-7)   | Seg 0 | Seg 1 | ... | Seg 127 | (128 Columns)
// |                     | Byte0 | Byte1 | ... | ByteN   |
// +---------------------------------------------------------------+
// | Page 1 (Rows 8-15)  | ...   | ...   | ... | ...     |
// ...
// | Page 7 (Rows 56-63) | ...   | ...   | ... | ...     |
// +---------------------------------------------------------------+
//
// - 1 Segment (Column) corresponds to 1 Pixel of WIDTH.
// - The Data Byte written to that Segment controls 8 Pixels of HEIGHT.
// LSB (Bit 0) is the top pixel in the page.
// MSB (Bit 7) is the bottom pixel in the page.

/**
 * Initializes the OLED display.
 * Performs the power-on sequence (VDD -> Wait -> VCC) and sends configuration commands.
 * @param SpiInst Pointer to the initialized SPI Master driver instance.
 * @param GpioInst Pointer to the initialized GPIO driver instance.
 * @return SPI_AXI_SUCCESS on success, error code otherwise.
 */
SPI_Status OLED_Init(SPI_Master_Driver *SpiInst, GPIO_Driver *GpioInst);

/**
 * Clears the OLED display buffer (writes 0 to all pixels).
 * @return SPI_AXI_SUCCESS on success.
 */
SPI_Status OLED_Clear(void);

/**
 * Sets the cursor position.
 * @param Page Page address (0-7). Each page is 8 pixels high.
 * @param Column Column address (0-127).
 * @return SPI_AXI_SUCCESS on success.
 */
SPI_Status OLED_SetCursor(u8 Page, u8 Column);

/**
 * Writes a single character to the display at the current cursor position.
 * @param c The ASCII character to write.
 * @return SPI_AXI_SUCCESS on success.
 */
SPI_Status OLED_PutChar(char c);

/**
 * Writes a null-terminated string to the display.
 * @param str Pointer to the string.
 * @return SPI_AXI_SUCCESS on success.
 */
SPI_Status OLED_PutString(const char *str);

/**
 * Writes a null-terminated string to the display using the 8x8 font.
 * @param str Pointer to the string.
 * @return SPI_AXI_SUCCESS on success.
 */
SPI_Status OLED_PutString8x8(const char *str);

#endif /* OLED_DRIVER_H_ */
