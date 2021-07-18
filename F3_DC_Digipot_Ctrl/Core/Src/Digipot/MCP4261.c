/* MCP4261 Library, for driving the 8-Bit Single/Dual SPI Digital POT with Non-Volatile Memory
 *
 * Originally created by by Steen Joergensen (stjo2809)
 * Converted C++ -> C with HAL by catzcod
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

#include "MCP4261.h"

static SPI_HandleTypeDef *HSPI; // SPI HAL handle

//=============================================================================
// Private functions
//=============================================================================

uint8_t MCP4261_makecommand_byte(uint16_t com, uint8_t address, uint16_t data) {
	uint8_t command_byte;

	if (data > 0xff && data <= 0x3FF) {
		command_byte = address << 4;            // add address to command_byte
		command_byte = command_byte | (data >> 8); // add data to command_byte
		command_byte = command_byte | (com << 2);  // add com to command_byte
	} else {
		command_byte = address << 4;            // add address to command_byte
		command_byte = command_byte | (com << 2);  // add com to command_byte
	}

	return command_byte;
}

//=============================================================================
// Public functions
//=============================================================================

void MCP4261_Init(SPI_HandleTypeDef *hspi) {
	HSPI = hspi;
	SHDN_PIN_SET;
	WP_PIN_SET;
	CS_PIN_SET;
}

uint16_t MCP4261_read(uint8_t address) {
	uint8_t response_msb;
	uint8_t response_lsb;
	uint16_t response;
	uint8_t command_byte;

	response = 0;                           // clear response for old data
	response_msb = 0;                       // clear response_msb for old data
	response_lsb = 0;                       // clear response_lsb for old data

	command_byte = MCP4261_makecommand_byte(CB_READ, address, 0);
	CS_PIN_RESET;
	HAL_SPI_TransmitReceive(HSPI, &command_byte, &response_msb,
			sizeof(command_byte), 1000);
	command_byte = 0xff; // not important bit of the 16 bits
	HAL_SPI_TransmitReceive(HSPI, &command_byte, &response_lsb,
			sizeof(command_byte), 1000);
	CS_PIN_SET;

	response = response_msb << 8;
	response = response | response_lsb;

	return response;
}

void MCP4261_write(uint8_t address, uint16_t data) {
	uint8_t command_byte = MCP4261_makecommand_byte(CB_WRITE, address, data);
	uint16_t send_data = data & 0xff;

	CS_PIN_RESET;
	HAL_SPI_Transmit(HSPI, &command_byte, sizeof(command_byte), 1000);
	HAL_SPI_Transmit(HSPI, (uint8_t*) &send_data, sizeof(send_data), 1000);
	CS_PIN_SET;
}

void MCP4261_inc(uint8_t number) {
	uint8_t command_byte;
	if (number == 0) {
		command_byte = MCP4261_makecommand_byte(CB_INCR, VW0_ADDR, 0);
	} else {
		command_byte = MCP4261_makecommand_byte(CB_INCR, VW1_ADDR, 0);
	}
	CS_PIN_RESET;
	HAL_Delay(10);
	HAL_SPI_Transmit(HSPI, &command_byte, sizeof(command_byte), 1000);
	CS_PIN_SET;
}

void MCP4261_dec(uint8_t number) {
	uint8_t command_byte;
	if (number == 0) {
		command_byte = MCP4261_makecommand_byte(CB_DECR, VW0_ADDR, 0);
	} else {
		command_byte = MCP4261_makecommand_byte(CB_DECR, VW1_ADDR, 0);
	}
	CS_PIN_RESET;
	HAL_Delay(10);
	HAL_SPI_Transmit(HSPI, &command_byte, sizeof(command_byte), 1);
	CS_PIN_SET;
}

uint16_t MCP4261_status_read() {
	return MCP4261_read(STATUS_ADDR);
}

uint16_t MCP4261_tcon_read() {
	return MCP4261_read(TCON_ADDR);
}

void MCP4261_tcon_write(uint16_t data) {
	MCP4261_write(TCON_ADDR, data);
}

uint16_t MCP4261_wiper_read(uint8_t number) {
	if (number == 0) {
		return MCP4261_read(VW0_ADDR) & 0x01FF;
	} else {
		return MCP4261_read(VW1_ADDR) & 0x01FF;
	}
}

void MCP4261_wiper_write(uint8_t number, uint16_t data) {
	if (number == 0) {
		MCP4261_write(VW0_ADDR, data & 0x00FF);
	} else {
		MCP4261_write(VW1_ADDR, data & 0x00FF);
	}
}

uint16_t MCP4261_nvwiper_read(uint8_t number) {
	if (number == 0) {
		return MCP4261_read(NVW0_ADDR);
	} else {
		return MCP4261_read(NVW1_ADDR);
	}
}

void MCP4261_nvwiper_write(uint8_t number, uint16_t data) {
	if (number == 0) {
		MCP4261_write(NVW0_ADDR, data);
	} else {
		MCP4261_write(NVW1_ADDR, data);
	}
}

void MCP4261_shdn_activate(uint8_t act) {
	if (act == 0) {
		SHDN_PIN_SET;
	} else {
		SHDN_PIN_RESET;
	}
}

void MCP4261_wp_activate(uint8_t act) {
	if (act == 0) {
		WP_PIN_SET;
	} else {
		WP_PIN_RESET;
	}
}
