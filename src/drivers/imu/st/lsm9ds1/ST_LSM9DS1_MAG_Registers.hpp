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
 * @file ST_LSM9DS1_MAG_registers.hpp
 *
 * ST LSM9DS1 magnetometer registers.
 *
 */

#pragma once

#include <cstdint>

namespace ST_LSM9DS1_MAG
{

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t LSM9DS1_MAG_WHO_AM_I = 0b01101000; // Who I am ID

static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz SPI clock frequency

static constexpr uint32_t M_ODR = 80; // Magnetometer output data rate

enum class
Register : uint8_t {
	WHO_AM_I     = 0x0F,

	CTRL_REG1_M  = 0x20,
	CTRL_REG2_M  = 0x21,
	CTRL_REG3_M  = 0x22,

	STATUS_REG_M = 0x27,
	OUT_X_L_M    = 0x28,
	OUT_X_H_M    = 0x29,
	OUT_Y_L_M    = 0x2A,
	OUT_Y_H_M    = 0x2B,
	OUT_Z_L_M    = 0x2C,
	OUT_Z_H_M    = 0x2D,
};

// CTRL_REG1_M
enum
CTRL_REG1_M_BIT : uint8_t {
	OM_ULTRA_HIGH_PERFORMANCE = Bit6 | Bit5,
	DO_80HZ                   = Bit4 | Bit3 | Bit2,
};

// CTRL_REG2_M
enum
CTRL_REG2_M_BIT : uint8_t {
	SOFT_RST = Bit2,
};

// CTRL_REG3_M
enum
CTRL_REG3_M_BIT : uint8_t {
	I2C_DISABLE = Bit7,

	MD_CONTINUOUS_MODE = Bit1 | Bit0,
};

// STATUS_REG_M
enum
STATUS_REG_M_BIT : uint8_t {
	ZYXOR = Bit7, // X, Y and Z-axis data overrun.
	ZYXDA = Bit3, // X, Y and Z-axis new data available.
};

} // namespace ST_LSM9DS1_MAG
