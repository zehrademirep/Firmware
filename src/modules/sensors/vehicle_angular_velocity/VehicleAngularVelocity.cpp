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

#include "VehicleAngularVelocity.hpp"

#include <px4_log.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	Stop();
}

bool
VehicleAngularVelocity::Start()
{
	// initialize thermal corrections as we might not immediately get a topic update (only non-zero values)
	_scale = Vector3f{1.0f, 1.0f, 1.0f};
	_offset.zero();
	_bias.zero();

	// force initial updates
	ParametersUpdate(true);
	SensorBiasUpdate(true);

	// needed to change the active sensor if the primary stops updating
	_sensor_selection_sub.registerCallback();

	return SensorCorrectionsUpdate(true);
}

void
VehicleAngularVelocity::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}

	_sensor_selection_sub.unregisterCallback();
}

void
VehicleAngularVelocity::SensorBiasUpdate(bool force)
{
	if (_sensor_bias_sub.updated() || force) {
		sensor_bias_s bias;

		if (_sensor_bias_sub.copy(&bias)) {
			// TODO: should be checking device ID
			_bias = Vector3f{bias.gyro_bias};
		}
	}
}

bool
VehicleAngularVelocity::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {

		sensor_correction_s corrections{};
		_sensor_correction_sub.copy(&corrections);

		// find in sensor corrections
		if ((_sensor_correction_index < 0)
		    || (corrections.gyro_device_id[_sensor_correction_index] != _selected_sensor_device_id)) {

			for (int correction_index = 0; correction_index < MAX_SENSOR_COUNT; correction_index++) {
				if (corrections.gyro_device_id[correction_index] == _selected_sensor_device_id) {
					_sensor_correction_index = correction_index;
				}
			}
		}

		if (_sensor_correction_index == 0) {
			_offset = Vector3f{corrections.gyro_offset_0};
			_scale = Vector3f{corrections.gyro_scale_0};

		} else if (_sensor_correction_index == 1) {
			_offset = Vector3f{corrections.gyro_offset_1};
			_scale = Vector3f{corrections.gyro_scale_1};

		} else if (_sensor_correction_index == 2) {
			_offset = Vector3f{corrections.gyro_offset_2};
			_scale = Vector3f{corrections.gyro_scale_2};

		} else {
			_offset = Vector3f{0.0f, 0.0f, 0.0f};
			_scale = Vector3f{1.0f, 1.0f, 1.0f};
		}
	}

	return false;
}

bool
VehicleAngularVelocity::SensorSelectionUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_selection_sub.updated() || force) {

		sensor_selection_s selection{};
		_sensor_selection_sub.copy(&selection);

		// update the latest sensor selection
		if ((_selected_sensor_device_id != selection.gyro_device_id) || force) {

			// clear all registered callbacks
			for (auto &sub : _sensor_control_sub) {
				sub.unregisterCallback();
			}

			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			// subscribe to sensor_gyro_control if available
			//  currently not all drivers (eg df_*) provide sensor_gyro_control
			for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
				sensor_gyro_control_s data{};
				_sensor_control_sub[sensor_index].copy(&data);

				if (data.device_id == _selected_sensor_device_id) {
					if (_sensor_control_sub[sensor_index].registerCallback()) {
						PX4_DEBUG("selected sensor changed %d -> %d (%d)", _selected_sensor_index, sensor_index, selection.gyro_device_id);

						_selected_sensor_device_id = selection.gyro_device_id;
						_selected_sensor_index = sensor_index;
						_sensor_correction_index = -1; // reset

						_sensor_control_available = true;

						SensorCorrectionsUpdate(true);

						return true;
					}
				}
			}

			// otherwise fallback to using sensor_gyro (legacy that will be removed)
			_sensor_control_available = false;

			for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
				sensor_gyro_s data{};
				_sensor_sub[sensor_index].copy(&data);

				if (data.device_id == selection.gyro_device_id) {
					if (_sensor_sub[sensor_index].registerCallback()) {
						PX4_DEBUG("selected sensor changed %d -> %d (%d)", _selected_sensor_index, sensor_index, selection.gyro_device_id);

						_selected_sensor_device_id = selection.gyro_device_id;
						_selected_sensor_index = sensor_index;
						_sensor_correction_index = -1; // reset

						SensorCorrectionsUpdate(true);

						return true;
					}
				}
			}
		}
	}

	return false;
}

void
VehicleAngularVelocity::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		// get transformation matrix from sensor/board to body frame
		const Dcmf board_rotation = get_rot_matrix((enum Rotation)_param_sens_board_rot.get());

		// fine tune the rotation
		const Dcmf board_rotation_offset(Eulerf(
				radians(_param_sens_board_x_off.get()),
				radians(_param_sens_board_y_off.get()),
				radians(_param_sens_board_z_off.get())));

		_board_rotation = board_rotation_offset * board_rotation;
	}
}

void
VehicleAngularVelocity::Run()
{
	// check for updated selected sensor
	SensorSelectionUpdate();

	if (_sensor_control_available) {
		//  using sensor_gyro_control is preferred, but currently not all drivers (eg df_*) provide sensor_gyro_control
		sensor_gyro_control_s sensor_data;

		if (_sensor_control_sub[_selected_sensor_index].update(&sensor_data)) {
			ParametersUpdate();
			SensorBiasUpdate();
			SensorCorrectionsUpdate();

			// get the sensor data and correct for thermal errors (apply offsets and scale)
			Vector3f rates{(Vector3f{sensor_data.xyz} - _offset).emult(_scale)};

			// rotate corrected measurements from sensor to body frame
			rates = _board_rotation * rates;

			// correct for in-run bias errors
			rates -= _bias;

			vehicle_angular_velocity_s angular_velocity;
			angular_velocity.timestamp_sample = sensor_data.timestamp_sample;
			rates.copyTo(angular_velocity.xyz);
			angular_velocity.timestamp = hrt_absolute_time();

			_vehicle_angular_velocity_pub.publish(angular_velocity);
		}

	} else {
		// otherwise fallback to using sensor_gyro (legacy that will be removed)
		sensor_gyro_s sensor_data;

		if (_sensor_sub[_selected_sensor_index].update(&sensor_data)) {
			ParametersUpdate();
			SensorBiasUpdate();
			SensorCorrectionsUpdate();

			// get the sensor data and correct for thermal errors
			const Vector3f val{sensor_data.x, sensor_data.y, sensor_data.z};

			// apply offsets and scale
			Vector3f rates{(val - _offset).emult(_scale)};

			// rotate corrected measurements from sensor to body frame
			rates = _board_rotation * rates;

			// correct for in-run bias errors
			rates -= _bias;

			vehicle_angular_velocity_s angular_velocity;
			angular_velocity.timestamp_sample = sensor_data.timestamp;
			rates.copyTo(angular_velocity.xyz);
			angular_velocity.timestamp = hrt_absolute_time();

			_vehicle_angular_velocity_pub.publish(angular_velocity);
		}
	}
}

void
VehicleAngularVelocity::PrintStatus()
{
	PX4_INFO_RAW("\n");

	if (_sensor_control_available) {
		PX4_INFO("selected sensor: %d (sensor_gyro_control:%d)", _selected_sensor_device_id, _selected_sensor_index);

	} else {
		PX4_WARN("sensor_gyro_control unavailable for selected sensor: %d (sensor_gyro:%d)", _selected_sensor_device_id,
			 _selected_sensor_index);
	}

	PX4_INFO("offsets: %.5f %.5f %.5f", (double)_offset(0), (double)_offset(1), (double)_offset(2));
	PX4_INFO("scale: %.5f %.5f %.5f", (double)_scale(0), (double)_scale(1), (double)_scale(2));
	PX4_INFO("bias: %.5f %.5f %.5f", (double)_bias(0), (double)_bias(1), (double)_bias(2));
}
