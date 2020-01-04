/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ST_LSM9DS1_registers.hpp
 *
 * ST LSM9DS1 registers.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace ST_LSM9DS1
{

static constexpr uint8_t DIR_READ	= 0x80;

static constexpr uint8_t LSM9DS1_WHO_AM_I = 0b01101000; // Who I am ID

static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz SPI clock frequency

static constexpr uint32_t LA_ODR = 952; // Linear acceleration output data rate
static constexpr uint32_t G_ODR = 952;  // Angular rate output data rate

enum class
Register : uint8_t {
	WHO_AM_I        = 0x0F,

	CTRL_REG1_G     = 0x10,	// Angular rate sensor Control Register 1.

	OUT_TEMP_L      = 0x15,
	OUT_TEMP_H      = 0x16,

	OUT_X_L_G       = 0x18,
	OUT_X_H_G       = 0x19,
	OUT_Y_L_G       = 0x1A,
	OUT_Y_H_G       = 0x1B,
	OUT_Z_L_G       = 0x1C,
	OUT_Z_H_G       = 0x1D,

	CTRL_REG6_XL    = 0x20, // Linear acceleration sensor Control Register 6.

	CTRL_REG8       = 0x22, // Control register 8.
	CTRL_REG9       = 0x23, // Control register 9.

	OUT_X_L_XL      = 0x28,
	OUT_X_H_XL      = 0x29,
	OUT_Y_L_XL      = 0x2A,
	OUT_Y_H_XL      = 0x2B,
	OUT_Z_L_XL      = 0x2C,
	OUT_Z_H_XL      = 0x2D,
	FIFO_CTRL       = 0x2E, // FIFO control register.
	FIFO_SRC        = 0x2F, // FIFO status control register.
};

// CTRL_REG1_G
enum
CTRL_REG1_G_BIT : uint8_t {
	ODR_G_952HZ  = Bit7 | Bit6,
	FS_G_2000DPS = Bit4 | Bit3,
	BW_G_100Hz   = Bit1 | Bit2, // BW_G 100 Hz
};

// CTRL_REG6_XL
enum
CTRL_REG6_XL_BIT : uint8_t {
	ODR_XL_952HZ = Bit7 | Bit6,	// 952 Hz ODR
	FS_XL_16     = Bit3,		// FS_XL 01: ±16 g
};

// CTRL_REG8
enum
CTRL_REG8_BIT : uint8_t {
	SW_RESET = Bit0,
};

// CTRL_REG9
enum
CTRL_REG9_BIT : uint8_t {
	I2C_DISABLE = Bit2,
	FIFO_EN     = Bit1,
};

// FIFO_CTRL
enum
FIFO_CTRL_BIT : uint8_t {
	FIFO_MODE_CONTINUOUS = Bit7 | Bit6, // Continuous mode. If the FIFO is full, the new sample over-writes the older sample.
};

// FIFO_SRC
enum
FIFO_SRC_BIT : uint8_t {
	OVRN = Bit6, // FIFO overrun status.
	FSS  = Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};


namespace FIFO
{
static constexpr size_t SIZE = 64;

// Saving data in the FIFO buffer is organized in four FIFO data sets consisting of 6 bytes each
// each FIFO sample is composed of 16 bits
struct DATA {
	uint8_t OUTX_L_G;
	uint8_t OUTX_H_G;
	uint8_t OUTY_L_G;
	uint8_t OUTY_H_G;
	uint8_t OUTZ_L_G;
	uint8_t OUTZ_H_G;

	uint8_t OUTX_L_XL;
	uint8_t OUTX_H_XL;
	uint8_t OUTY_L_XL;
	uint8_t OUTY_H_XL;
	uint8_t OUTZ_L_XL;
	uint8_t OUTZ_H_XL;
};
static_assert(sizeof(DATA) == 12);
}

} // namespace ST_LSM9DS1
