/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/device/integrator.h>
#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>

class PX4Accelerometer : public ModuleParams
{

public:
	PX4Accelerometer(uint32_t device_id, uint8_t priority = ORB_PRIO_DEFAULT, enum Rotation rotation = ROTATION_NONE);
	~PX4Accelerometer() override = default;

	void set_device_type(uint8_t devtype);
	void set_error_count(uint64_t error_count) { _error_count = error_count; }
	void set_sample_rate(unsigned rate);
	void set_scale(float scale) { _scaling = scale; }
	void set_temperature(float temperature) { _temperature = temperature; }

	void update(hrt_abstime timestamp_sample, float x, float y, float z);

	void print_status();

private:
	void set_filter_cutoff(float cutoff_freq);

	void UpdateCalibration();

	uORB::PublicationMulti<sensor_accel_s>	_sensor_pub;

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	math::LowPassFilter2pVector3f _filter{1000, 100};
	Integrator _integrator{4000, false};

	uint64_t		_error_count{0};
	uint32_t		_device_id{0};
	const enum Rotation	_rotation;
	float			_scaling{1.0f};
	matrix::Vector3f	_calibration_scale{1.0f, 1.0f, 1.0f};
	matrix::Vector3f	_calibration_offset{0.0f, 0.0f, 0.0f};
	unsigned		_sample_rate{1000};
	float			_temperature{0.0f};

	bool			_calibrated{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IMU_ACCEL_CUTOFF>) _param_imu_accel_cutoff
	)

};
