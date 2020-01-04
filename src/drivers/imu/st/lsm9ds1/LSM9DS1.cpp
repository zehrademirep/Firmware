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

#include "LSM9DS1.hpp"

#include <px4_platform/board_dma_alloc.h>

using namespace time_literals;
using namespace ST_LSM9DS1;

static constexpr int16_t combine(uint8_t lsb, uint8_t msb) { return (msb << 8u) | lsb; }

LSM9DS1::LSM9DS1(int bus, uint32_t device, enum Rotation rotation) :
	SPI(MODULE_NAME, nullptr, bus, device, SPIDEV_MODE3, SPI_SPEED),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), ORB_PRIO_DEFAULT, rotation),
	_px4_gyro(get_device_id(), ORB_PRIO_DEFAULT, rotation)
{
	set_device_type(DRV_DEVTYPE_ST_LSM9DS1);
	_px4_accel.set_device_type(DRV_DEVTYPE_ST_LSM9DS1);
	_px4_gyro.set_device_type(DRV_DEVTYPE_ST_LSM9DS1);

	_px4_accel.set_sample_rate(ST_LSM9DS1::LA_ODR);
	_px4_gyro.set_sample_rate(ST_LSM9DS1::G_ODR);

	_px4_accel.set_update_rate(1000000 / _fifo_interval);
	_px4_gyro.set_update_rate(1000000 / _fifo_interval);
}

LSM9DS1::~LSM9DS1()
{
	Stop();

	if (_dma_data_buffer != nullptr) {
		board_dma_free(_dma_data_buffer, FIFO::SIZE);
	}

	perf_free(_interval_perf);
	perf_free(_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
}

int LSM9DS1::probe()
{
	if (RegisterRead(Register::WHO_AM_I) == LSM9DS1_WHO_AM_I) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

bool LSM9DS1::Init()
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
	_dma_data_buffer = (uint8_t *)board_dma_alloc(FIFO::SIZE);

	if (_dma_data_buffer == nullptr) {
		PX4_ERR("DMA alloc failed");
		return false;
	}

	Start();

	return true;
}

bool LSM9DS1::Reset()
{
	for (int i = 0; i < 5; i++) {
		// Reset
		// CTRL_REG8: SW_RESET
		// !!!!!!!!!!!! Note: When the FIFO is used, the IF_INC and BDU bits must be equal to 1.
		RegisterSetBits(Register::CTRL_REG8, CTRL_REG8_BIT::SW_RESET);
		usleep(50); // Wait 50 μs (or wait until the SW_RESET bit of the CTRL_REG8 register returns to 0).

		RegisterSetBits(Register::CTRL_REG9, CTRL_REG9_BIT::I2C_DISABLE);

		// Gyroscope configuration
		// CTRL_REG1_G: Gyroscope 2000 degrees/second and ODR 952 Hz
		RegisterWrite(Register::CTRL_REG1_G,
			      CTRL_REG1_G_BIT::ODR_G_952HZ | CTRL_REG1_G_BIT::FS_G_2000DPS | CTRL_REG1_G_BIT::BW_G_100Hz);
		_px4_gyro.set_scale(math::radians(70.0f / 1000.0f)); // 70 mdps/LSB
		_px4_gyro.set_range(math::radians(2000.0f));

		// Accelerometer configuration
		// CTRL_REG6_XL: Accelerometer 16 G range and ODR 952 Hz
		RegisterWrite(Register::CTRL_REG6_XL, CTRL_REG6_XL_BIT::ODR_XL_952HZ | CTRL_REG6_XL_BIT::FS_XL_16);
		_px4_accel.set_scale(0.732f * (CONSTANTS_ONE_G / 1000.0f));	// 0.732 mg/LSB
		_px4_accel.set_range(16.0f * CONSTANTS_ONE_G);


		const bool reset_done = ((RegisterRead(Register::CTRL_REG8) & CTRL_REG8_BIT::SW_RESET) == 0);

		// reset done once data is ready
		if (reset_done) {
			return true;
		}
	}

	return false;
}

void LSM9DS1::ResetFIFO()
{
	perf_count(_fifo_reset_perf);

	// CTRL_REG9 - disable FIFO
	RegisterClearBits(Register::CTRL_REG9, CTRL_REG9_BIT::FIFO_EN);

	// FIFO_CTRL: FIFO continuous mode enabled
	RegisterWrite(Register::FIFO_CTRL, FIFO_CTRL_BIT::FIFO_MODE_CONTINUOUS);

	// CTRL_REG9 - enable FIFO
	RegisterSetBits(Register::CTRL_REG9, CTRL_REG9_BIT::FIFO_EN);
}

uint8_t LSM9DS1::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void LSM9DS1::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void LSM9DS1::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void LSM9DS1::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

void LSM9DS1::Start()
{
	Stop();

	ResetFIFO();

	ScheduleOnInterval(_fifo_interval, _fifo_interval);
}

void LSM9DS1::Stop()
{
	ScheduleClear();
}

void LSM9DS1::Run()
{
	perf_count(_interval_perf);

	// Number of unread words (16-bit axes) stored in FIFO.
	const hrt_abstime timestamp_fifo_level = hrt_absolute_time();
	const uint8_t FIFO_SRC = RegisterRead(Register::FIFO_SRC);

	if (FIFO_SRC & FIFO_SRC_BIT::OVRN) {
		// overflow
		perf_count(_fifo_overflow_perf);
		ResetFIFO();
		return;
	}

	const uint8_t samples = FIFO_SRC & !static_cast<uint8_t>(FIFO_SRC_BIT::FSS);

	if (samples < 1) {
		perf_count(_fifo_empty_perf);
		return;

	} else if (samples > 32) {
		// not technically an overflow, but more samples than we expected
		perf_count(_fifo_overflow_perf);
		ResetFIFO();
		return;
	}

	static constexpr uint32_t gyro_dt = 1000000 / ST_LSM9DS1::G_ODR;

	// estimate timestamp of first sample in the FIFO from number of samples and fill rate
	const hrt_abstime timestamp_sample = timestamp_fifo_level - ((samples - 1) * gyro_dt);

	PX4Accelerometer::FIFOSample accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = samples;
	accel.dt = gyro_dt;

	PX4Gyroscope::FIFOSample gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = gyro_dt;

	perf_begin(_transfer_perf);

	for (int i = 0; i < samples; i++) {
		// Gyro
		struct GyroReport {
			uint8_t cmd;
			uint8_t OUT_X_L_G;
			uint8_t OUT_X_H_G;
			uint8_t OUT_Y_L_G;
			uint8_t OUT_Y_H_G;
			uint8_t OUT_Z_L_G;
			uint8_t OUT_Z_H_G;
		};
		GyroReport *greport = (GyroReport *)_dma_data_buffer;
		memset(greport, 0, sizeof(GyroReport));
		greport->cmd = static_cast<uint8_t>(Register::OUT_X_L_G) | DIR_READ;

		if (transfer(_dma_data_buffer, _dma_data_buffer, sizeof(GyroReport)) != PX4_OK) {
			perf_end(_transfer_perf);
			return;
		}

		// sensor Z is up (RHC), flip z for publication
		gyro.x[i] = combine(greport->OUT_X_L_G, greport->OUT_X_H_G);
		gyro.y[i] = combine(greport->OUT_Y_L_G, greport->OUT_Y_H_G);
		gyro.z[i] = -combine(greport->OUT_Z_L_G, greport->OUT_Z_H_G);


		// Accel
		struct AccelReport {
			uint8_t cmd;
			uint8_t OUT_X_L_XL;
			uint8_t OUT_X_H_XL;
			uint8_t OUT_Y_L_XL;
			uint8_t OUT_Y_H_XL;
			uint8_t OUT_Z_L_XL;
			uint8_t OUT_Z_H_XL;
		};
		AccelReport *areport = (AccelReport *)_dma_data_buffer;
		memset(areport, 0, sizeof(GyroReport));
		areport->cmd = static_cast<uint8_t>(Register::OUT_X_L_XL) | DIR_READ;

		if (transfer(_dma_data_buffer, _dma_data_buffer, sizeof(AccelReport)) != PX4_OK) {
			perf_end(_transfer_perf);
			return;
		}

		// sensor Z is up (RHC), flip z for publication
		accel.x[i] = combine(areport->OUT_X_L_XL, areport->OUT_X_H_XL);
		accel.y[i] = combine(areport->OUT_Y_L_XL, areport->OUT_Y_H_XL);
		accel.z[i] = -combine(areport->OUT_Z_L_XL, areport->OUT_Z_H_XL);
	}

	perf_end(_transfer_perf);

	// get current temperature at 1 Hz
	if (hrt_elapsed_time(&_time_last_temperature_update) > 1_s) {
		uint8_t temperature_buf[3] {};
		temperature_buf[0] = static_cast<uint8_t>(Register::OUT_TEMP_L) | DIR_READ;

		if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
			return;
		}

		// 16 bits in two’s complement format with a sensitivity of 256 LSB/°C. The output zero level corresponds to 25 °C.
		const int16_t OUT_TEMP = combine(temperature_buf[1], temperature_buf[2] & 0x0F);
		const float temperature = (OUT_TEMP / 256.0f) + 25.0f;

		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}

	_px4_gyro.updateFIFO(gyro);
	_px4_accel.updateFIFO(accel);
}

void LSM9DS1::PrintInfo()
{
	perf_print_counter(_interval_perf);
	perf_print_counter(_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);

	_px4_accel.print_status();
	_px4_gyro.print_status();
}
