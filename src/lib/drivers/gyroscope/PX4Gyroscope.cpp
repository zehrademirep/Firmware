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


#include "PX4Gyroscope.hpp"

#include <lib/drivers/device/Device.hpp>

PX4Gyroscope::PX4Gyroscope(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	ModuleParams(nullptr),
	_sensor_pub{ORB_ID(sensor_gyro), priority},
	_sensor_control_pub{ORB_ID(sensor_gyro_control), priority},
	_device_id(device_id),
	_rotation{rotation}
{
	// set software low pass filter for controllers
	updateParams();
	UpdateCalibration();

	set_filter_cutoff(_param_imu_gyro_cutoff.get());
}

void
PX4Gyroscope::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back
	_device_id = device_id.devid;

	UpdateCalibration();
}

void
PX4Gyroscope::set_filter_cutoff(float cutoff)
{
	// only do expensive filter update if the cutoff changed
	if (fabsf(_filter.get_cutoff_freq() - cutoff) > 0.01f) {
		_filter.set_cutoff_frequency(_sample_rate, cutoff);
	}
}

void
PX4Gyroscope::set_sample_rate(unsigned rate)
{
	_sample_rate = rate;
	_filter.set_cutoff_frequency(_sample_rate, _filter.get_cutoff_freq());
}

void
PX4Gyroscope::UpdateCalibration()
{
	for (unsigned i = 0; i < 3; ++i) {
		char str[30] {};
		sprintf(str, "CAL_GYRO%u_ID", i);
		int32_t device_id = 0;

		if (param_get(param_find(str), &device_id) != OK) {
			PX4_ERR("Could not access param %s", str);
			continue;
		}

		if (device_id == 0) {
			continue;
		}

		if ((uint32_t)device_id == _device_id) {

			// scale factors (x, y, z)
			float scale[3] {};

			sprintf(str, "CAL_GYRO%u_XSCALE", i);
			param_get(param_find(str), &scale[0]);

			sprintf(str, "CAL_GYRO%u_YSCALE", i);
			param_get(param_find(str), &scale[1]);

			sprintf(str, "CAL_GYRO%u_ZSCALE", i);
			param_get(param_find(str), &scale[2]);

			_calibration_scale = matrix::Vector3f{scale};


			// offsets factors (x, y, z)
			float offset[3] {};

			sprintf(str, "CAL_GYRO%u_XOFF", i);
			param_get(param_find(str), &offset[0]);

			sprintf(str, "CAL_GYRO%u_YOFF", i);
			param_get(param_find(str), &offset[1]);

			sprintf(str, "CAL_GYRO%u_ZOFF", i);
			param_get(param_find(str), &offset[2]);

			_calibration_offset = matrix::Vector3f{offset};

			_calibrated = true;

			return;
		}
	}

	// reset if no calibration data found
	_calibration_scale = matrix::Vector3f{1.0f, 1.0f, 1.0f};
	_calibration_offset.zero();
	_calibrated = false;
}

void
PX4Gyroscope::update(hrt_abstime timestamp_sample, float x, float y, float z)
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		UpdateCalibration();

		set_filter_cutoff(_param_imu_gyro_cutoff.get());
	}

	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	const matrix::Vector3f raw{x, y, z};

	// Apply range scale and the calibrating offset/scale
	const matrix::Vector3f val_calibrated{(((raw * _scaling) - _calibration_offset).emult(_calibration_scale))};

	// Filtered values
	const matrix::Vector3f val_filtered{_filter.apply(val_calibrated)};


	// publish control data (filtered gyro) immediately
	bool publish_control = true;

	if (_param_imu_gyro_rate_max.get() > 0) {
		const uint64_t interval = 1e6f / _param_imu_gyro_rate_max.get();

		if (hrt_elapsed_time(&_last_control_publish) < interval) {
			publish_control = false;
		}
	}

	if (publish_control) {
		sensor_gyro_control_s control{};
		control.timestamp_sample = timestamp_sample;
		control.device_id = _device_id;
		val_filtered.copyTo(control.xyz);
		control.timestamp = hrt_absolute_time();
		_sensor_control_pub.publish(control);

		_last_control_publish = control.timestamp;
	}

	// Integrated values
	matrix::Vector3f integrated_value;
	uint32_t integral_dt = 0;

	if (_integrator.put(timestamp_sample, val_calibrated, integrated_value, integral_dt)) {

		sensor_gyro_s report{};
		report.timestamp = timestamp_sample;
		report.error_count = _error_count;
		report.device_id = _device_id;
		report.temperature = _temperature;
		report.scaling = _scaling;

		report.x = val_filtered(0);
		report.y = val_filtered(1);
		report.z = val_filtered(2);

		report.integral_dt = integral_dt;
		report.x_integral = integrated_value(0);
		report.y_integral = integrated_value(1);
		report.z_integral = integrated_value(2);

		// Raw values (ADC units 0 - 65535)
		report.x_raw = x;
		report.y_raw = y;
		report.z_raw = z;

		_sensor_pub.publish(report);
	}
}

void
PX4Gyroscope::print_status()
{
	char device_id_buffer[80] {};
	device::Device::device_id_print_buffer(device_id_buffer, sizeof(device_id_buffer), _device_id);
	PX4_INFO("device id: %d (%s)", _device_id, device_id_buffer);
	PX4_INFO("rotation: %d", _rotation);
	PX4_INFO("sample rate: %d Hz", _sample_rate);
	PX4_INFO("filter cutoff: %.3f Hz", (double)_filter.get_cutoff_freq());
	PX4_INFO("calibrated: %d", _calibrated);

	PX4_INFO("calibration scale: %.5f %.5f %.5f", (double)_calibration_scale(0), (double)_calibration_scale(1),
		 (double)_calibration_scale(2));
	PX4_INFO("calibration offset: %.5f %.5f %.5f", (double)_calibration_offset(0), (double)_calibration_offset(1),
		 (double)_calibration_offset(2));

}
