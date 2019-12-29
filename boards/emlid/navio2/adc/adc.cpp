/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <board_config.h>
#include <stdint.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>

#define ADC_SYSFS_PATH "/sys/kernel/rcio/adc/ch0"
#define ADC_MAX_CHAN 6

/*
 * ADC Channels:
 * A0 - Board voltage (5V)
 * A1 - servo rail voltage
 * A2 - power module voltage (ADC0, POWER port)
 * A3 - power module current (ADC1, POWER port)
 * A4 - ADC2 (ADC port)
 * A5 - ADC3 (ADC port)
 */

#define NAVIO_ADC_BATTERY_VOLTAGE_CHANNEL (2)
#define NAVIO_ADC_BATTERY_CURRENT_CHANNEL (3)

int _fd[ADC_MAX_CHAN] {};

int px4_arch_adc_init(uint32_t base_address)
{
	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		char channel_path[sizeof(ADC_SYSFS_PATH)];
		strncpy(channel_path, ADC_SYSFS_PATH, sizeof(ADC_SYSFS_PATH));
		channel_path[sizeof(ADC_SYSFS_PATH) - 2] += i;

		_fd[i] = ::open(channel_path, O_RDONLY);

		if (_fd[i] == -1) {
			int err = errno;
			ret = -1;
			PX4_ERR("init: open: %s (%d)", strerror(err), err);
			goto cleanup;
		}
	}

	return PX4_OK;

cleanup:

	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		if (_fd[i] != -1) {
			::close(_fd[i]);
		}
	}

	return ret;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		if (_fd[i] != -1) {
			::close(_fd[i]);
		}
	}
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	char buffer[11] {}; /* 32bit max INT has maximum 10 chars */
	int ret = PX4_ERROR;

	if (channel < 0 || channel >= ADC_MAX_CHAN) {
		return -EINVAL;
	}

	ret = lseek(_fd[channel], 0, SEEK_SET);

	if (ret == -1) {
		ret = errno;
		PX4_ERR("read_channel %d: lseek: %s (%d)", channel, strerror(ret), ret);
		return  ret;
	}

	ret = ::read(_fd[channel], buffer, sizeof(buffer) - 1);

	if (ret == -1) {
		ret = errno;
		PX4_ERR("read_channel %d: read: %s (%d)", channel, strerror(ret), ret);
		return ret;

	} else if (ret == 0) {
		PX4_ERR("read_channel %d: read empty", channel);
		ret = -EINVAL;
		return ret;
	}

	buffer[ret] = 0;

	ret = 0;

	return ret;
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 1 << 16;
}

uint32_t px4_arch_adc_dn_fullcount(void)
{
	return 1 << 16; // 16 bit ADC
}
