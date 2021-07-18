/* MCP4261 Library, for driving the 8-Bit Single/Dual SPI Digital POT with Non-Volatile Memory
 *
 * Originally created by by Steen Joergensen (stjo2809)
 * Converted C++ -> C (HAL) and updated by catzcod
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//=============================================================================
// All The Addresses
//=============================================================================
// The device memory is 16 locations that are 9-bits wide (16x9 bits).
// This memory space contains both volatile and non-volatile locations.
#define VW0_ADDR        0x00       // Volatile Wiper 0 (RAM)
#define VW1_ADDR        0x01       // Volatile Wiper 1 (RAM)
#define NVW0_ADDR       0x02       // Non Volatile Wiper 0 (EEPROM)
#define NVW1_ADDR       0x03       // Non Volatile Wiper 1 (EEPROM)
#define TCON_ADDR       0x04       // Controls the state of each resistor network terminal connection. (RAM)
#define STATUS_ADDR     0x05       // Status (STATUS) Register, This register contains 5 status bits. WiperLock bits, Shutdown bit, Write Protect bit, EEPROM write cycle. (RAM)
#define EEPROMx06_ADDR  0x06       // Data (EEPROM)
#define EEPROMx07_ADDR  0x07       // Data (EEPROM)
#define EEPROMx08_ADDR  0x08       // Data (EEPROM)
#define EEPROMx09_ADDR  0x09       // Data (EEPROM)
#define EEPROMx0A_ADDR  0x0A       // Data (EEPROM)
#define EEPROMx0B_ADDR  0x0B       // Data (EEPROM)
#define EEPROMx0C_ADDR  0x0C       // Data (EEPROM)
#define EEPROMx0D_ADDR  0x0D       // Data (EEPROM)
#define EEPROMx0E_ADDR  0x0E       // Data (EEPROM)
#define EEPROMx0F_ADDR  0x0F       // Data (EEPROM)

// DATA EEPROM locations has the address from 0x06 to 0x0F 

//=============================================================================
// Declaration of variables & custom #defines
//=============================================================================

#define CB_WRITE        0x00       // Device command bit for WRITE
#define CB_INCR         0x01       // Device command bit for INCREMENT
#define CB_DECR         0x02       // Device command bit for DECREMENT
#define CB_READ         0x03       // Device command bit for READ

//=============================================================================
// Types Declaration
//=============================================================================
#include "main.h"

// ! ATTENTION ! Change GPIO port and pin according to your electrical layout!
#define CS_PIN_SET      HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)
#define CS_PIN_RESET    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define WP_PIN_SET      HAL_GPIO_WritePin(DPT_WP_GPIO_Port, DPT_WP_Pin, GPIO_PIN_SET)
#define WP_PIN_RESET    HAL_GPIO_WritePin(DPT_WP_GPIO_Port, DPT_WP_Pin, GPIO_PIN_RESET)
#define SHDN_PIN_SET    HAL_GPIO_WritePin(DPT_SHDN_GPIO_Port, DPT_SHDN_Pin, GPIO_PIN_SET)
#define SHDN_PIN_RESET  HAL_GPIO_WritePin(DPT_SHDN_GPIO_Port, DPT_SHDN_Pin, GPIO_PIN_RESET)
#define SPI_TIMEOUT     100

//=============================================================================
// Functions Declaration
//=============================================================================

/** Interface to the 7/8-Bit Single/Dual SPI Digital POT with Non-Volatile Memory
 *
 *  Using the driver:
 *   - remember to setup SPI in main routine or use pins instance.
 *
 *  Defaults in this driver on start up:
 *   - as default is HARDWARE WRITE PROTECT PIN "Off".
 *   - as default is HARDWARE SHUTDOWN PIN  "Off".
 *
 */
/** Create an instance of the MCP4261 connected via specfied SPI instance.
 *
 * @param spi The mbed SPI instance (make in main routine)
 * @param nWP The Hardware Write Protect Control pin.
 * @param nSHDN The Shutdown pin.
 * @param nCs The SPI chip select pin.
 */
void MCP4261_Init(SPI_HandleTypeDef *hspi);

/** Read an Address.
 *
 * @param address The selected register to read from.
 * @return The 16 bits read.
 */
uint16_t MCP4261_read(uint8_t address);

/** Write to Address.
 *
 * @param address The selected register to write to.
 * @param data The 16 bits to write to the register
 */
void MCP4261_write(uint8_t address, uint16_t data);

/** Increment wiper.
 *
 * @param number The selected wiper to increment.
 */
void MCP4261_inc(uint8_t number);

/** Decrement wiper.
 *
 * @param number The selected wiper to decrement.
 */
void MCP4261_dec(uint8_t number);

/** Read the Status register.
 *
 * @return The 16 bits read.
 */
uint16_t MCP4261_status_read();

/** Read the tcon register.
 *
 * @return The 16 bits read.
 */
uint16_t MCP4261_tcon_read();

/** write to tcon register.
 *
 * @param data The 16 bits to write to the register
 */
void MCP4261_tcon_write(uint16_t data);

/** Read the Volatile Wiper.
 *
 * @param number The wiper number = '0' or '1'
 * @return The 16 bits read.
 */
uint16_t MCP4261_wiper_read(uint8_t number);

/** write to Volatile Wiper.
 *
 * @param number The wiper number = '0' or '1'
 * @param data The 16 bits to write to the register
 */
void MCP4261_wiper_write(uint8_t number, uint16_t data);

/** Read the non-volatile wiper (Power On Reset start value).
 *
 * @param number The wiper number = '0' or '1'
 * @return The 16 bits read.
 */
uint16_t MCP4261_nvwiper_read(uint8_t number);

/** write to non-volatile wiper (Power On Reset start value).
 *
 * @param number The wiper number = '0' or '1'
 * @param data The 16 bits to write to the register
 */
void MCP4261_nvwiper_write(uint8_t number, uint16_t data);

/** HARDWARE SHUTDOWN PIN (SHDN)
 *
 * @param act SHDN is Active = true and Inactive = false
 */
void MCP4261_shdn_activate(uint8_t act);

/** HARDWARE WRITE PROTECT PIN (WP)
 *
 * @param act WP is Active = true and Inactive = false
 */
void MCP4261_wp_activate(uint8_t act);

