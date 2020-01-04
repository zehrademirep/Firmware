/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "LSM9DS1_MAG.hpp"

#include <px4_platform/board_dma_alloc.h>

using namespace time_literals;
using namespace ST_LSM9DS1_MAG;
using ST_LSM9DS1_MAG::Register;

static constexpr int16_t combine(uint8_t lsb, uint8_t msb) { return (msb << 8u) | lsb; }

LSM9DS1_MAG::LSM9DS1_MAG(int bus, uint32_t device, enum Rotation rotation) :
	SPI(MODULE_NAME, nullptr, bus, device, SPIDEV_MODE3, SPI_SPEED),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_mag(get_device_id(), ORB_PRIO_DEFAULT, rotation)
{
	set_device_type(DRV_DEVTYPE_ST_LSM9DS1);
	_px4_mag.set_device_type(DRV_DEVTYPE_ST_LSM9DS1);

	_px4_mag.set_temperature(NAN); // temperature not available
}

LSM9DS1_MAG::~LSM9DS1_MAG()
{
	Stop();

	if (_dma_data_buffer != nullptr) {
		board_dma_free(_dma_data_buffer, 8);
	}

	perf_free(_interval_perf);
	perf_free(_transfer_perf);
}

int LSM9DS1_MAG::probe()
{
	if (RegisterRead(Register::WHO_AM_I) == LSM9DS1_MAG_WHO_AM_I) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

bool LSM9DS1_MAG::Init()
{
	if (SPI::init() != PX4_OK) {
		PX4_ERR("SPI::init failed");
		return false;
	}

	if (!Reset()) {
		PX4_ERR("reset failed");
		return false;
	}

	// allocate DMA capable buffer
	_dma_data_buffer = (uint8_t *)board_dma_alloc(8);

	if (_dma_data_buffer == nullptr) {
		PX4_ERR("DMA alloc failed");
		return false;
	}

	Start();

	return true;
}

bool LSM9DS1_MAG::Reset()
{
	for (int i = 0; i < 5; i++) {
		// Reset
		// CTRL_REG2_M: SOFT_RST
		RegisterSetBits(Register::CTRL_REG2_M, CTRL_REG2_M_BIT::SOFT_RST);
		usleep(50);

		// Disable I2C interface
		// CTRL_REG3_M: I2C_DISABLE, Continuous-conversion mode
		RegisterSetBits(Register::CTRL_REG3_M, CTRL_REG3_M_BIT::I2C_DISABLE);
		RegisterClearBits(Register::CTRL_REG3_M, CTRL_REG3_M_BIT::MD_CONTINUOUS_MODE);

		// Magnetometer configuration
		// CTRL_REG1_M: Magnetometer ODR 80 Hz
		RegisterWrite(Register::CTRL_REG1_M, CTRL_REG1_M_BIT::OM_ULTRA_HIGH_PERFORMANCE | CTRL_REG1_M_BIT::DO_80HZ);
		_px4_mag.set_scale(0.14f); // Magnetic FS = ±4 gauss 0.14 mgauss/LSB

		const bool reset_done = ((RegisterRead(Register::CTRL_REG2_M) & CTRL_REG2_M_BIT::SOFT_RST) == 0);

		// reset done once data is ready
		if (reset_done) {
			return true;
		}
	}

	return false;
}

uint8_t LSM9DS1_MAG::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void LSM9DS1_MAG::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void LSM9DS1_MAG::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void LSM9DS1_MAG::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

void LSM9DS1_MAG::Start()
{
	Stop();

	ScheduleOnInterval(1000000 / ST_LSM9DS1_MAG::M_ODR);
}

void LSM9DS1_MAG::Stop()
{
	ScheduleClear();
}

void LSM9DS1_MAG::Run()
{
	perf_count(_interval_perf);

	struct Report {
		uint8_t cmd;
		uint8_t OUT_X_L_M;
		uint8_t OUT_X_H_M;
		uint8_t OUT_Y_L_M;
		uint8_t OUT_Y_H_M;
		uint8_t OUT_Z_L_M;
		uint8_t OUT_Z_H_M;
	};
	Report *report = (Report *)_dma_data_buffer;
	memset(report, 0, sizeof(Report));
	report->cmd = static_cast<uint8_t>(Register::STATUS_REG_M) | DIR_READ;

	perf_begin(_transfer_perf);
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (transfer(_dma_data_buffer, _dma_data_buffer, sizeof(Report)) != PX4_OK) {
		perf_end(_transfer_perf);
		return;
	}

	perf_end(_transfer_perf);

	// sensor Z is up (RHC), flip z for publication
	int16_t x = combine(report->OUT_X_L_M, report->OUT_X_H_M);
	int16_t y = combine(report->OUT_Y_L_M, report->OUT_Y_H_M);
	int16_t z = -combine(report->OUT_Z_L_M, report->OUT_Z_H_M);

	_px4_mag.update(timestamp_sample, x, y, z);
}

void LSM9DS1_MAG::PrintInfo()
{
	perf_print_counter(_interval_perf);
	perf_print_counter(_transfer_perf);

	_px4_mag.print_status();
}
