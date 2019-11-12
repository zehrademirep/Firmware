#! /usr/bin/env python3
"""
the ecl ekf analysis
"""

from typing import Tuple, List, Dict

import numpy as np
from pyulog import ULog

from analysis.detectors import InAirDetector, PreconditionError
from analysis.metrics import calculate_ecl_ekf_metrics
from analysis.checks import perform_ecl_ekf_checks
from analysis.post_processing import get_estimator_check_flags

def analyse_ekf(
        ulog: ULog, check_levels: Dict[str, float], red_thresh: float = 1.0,
        amb_thresh: float = 0.5, min_flight_duration_seconds: float = 5.0,
        in_air_margin_seconds: float = 5.0, pos_checks_when_sensors_not_fused: bool = False) -> \
        Tuple[str, Dict[str, str], Dict[str, float], Dict[str, float]]:
    """
    :param ulog:
    :param check_levels:
    :param red_thresh:
    :param amb_thresh:
    :param min_flight_duration_seconds:
    :param in_air_margin_seconds:
    :param pos_checks_when_sensors_not_fused:
    :return:
    """

    try:
        estimator_status = ulog.get_dataset('estimator_status').data
        print('found estimator_status data')
    except:
        raise PreconditionError('could not find estimator_status data')

    try:
        _ = ulog.get_dataset('ekf2_innovations').data
        print('found ekf2_innovation data')
    except:
        raise PreconditionError('could not find ekf2_innovation data')

    try:
        in_air = InAirDetector(
            ulog, min_flight_time_seconds=min_flight_duration_seconds, in_air_margin_seconds=0.0)
        in_air_no_ground_effects = InAirDetector(
            ulog, min_flight_time_seconds=min_flight_duration_seconds,
            in_air_margin_seconds=in_air_margin_seconds)
    except Exception as e:
        raise PreconditionError(str(e))

    if in_air_no_ground_effects.take_off is None:
        raise PreconditionError('no airtime detected.')

    airtime_info = {
        'in_air_transition_time': round(in_air.take_off + in_air.log_start, 2),
        'on_ground_transition_time': round(in_air.landing + in_air.log_start, 2)}

    control_mode, innov_flags, gps_fail_flags = get_estimator_check_flags(estimator_status)

    innov_msg_name = get_innovation_message(ulog)
    innov_var_msg_name = get_innovation_variance_message(ulog)
    field_name = get_field_name_from_message_and_descriptor("estimator_innovation_variances","drag_innovation_variance")
    print(innov_msg_name)
    print(innov_var_msg_name)
    print(field_name)

    sensor_checks, innov_fail_checks = find_checks_that_apply(
        control_mode, estimator_status,
        pos_checks_when_sensors_not_fused=pos_checks_when_sensors_not_fused)

    metrics = calculate_ecl_ekf_metrics(
        ulog, innov_flags, innov_fail_checks, sensor_checks, in_air, in_air_no_ground_effects,
        red_thresh=red_thresh, amb_thresh=amb_thresh)

    check_status, master_status = perform_ecl_ekf_checks(
        metrics, sensor_checks, innov_fail_checks, check_levels)

    return master_status, check_status, metrics, airtime_info


def find_checks_that_apply(
    control_mode: dict, estimator_status: dict, pos_checks_when_sensors_not_fused: bool = False) ->\
        Tuple[List[str], List[str]]:
    """
    finds the checks that apply and stores them in lists for the std checks and the innovation
    fail checks.
    :param control_mode:
    :param estimator_status:
    :param b_pos_only_when_sensors_fused:
    :return: a tuple of two lists that contain strings for the std checks and for the innovation
    fail checks.
    """
    sensor_checks = list()
    innov_fail_checks = list()

    # Height Sensor Checks
    sensor_checks.append('hgt')
    innov_fail_checks.append('posv')

    # Magnetometer Sensor Checks
    if (np.amax(control_mode['yaw_aligned']) > 0.5):
        sensor_checks.append('mag')

        innov_fail_checks.append('magx')
        innov_fail_checks.append('magy')
        innov_fail_checks.append('magz')
        innov_fail_checks.append('yaw')

    # Velocity Sensor Checks
    if (np.amax(control_mode['using_gps']) > 0.5):
        sensor_checks.append('vel')
        innov_fail_checks.append('vel')

    # Position Sensor Checks
    if (pos_checks_when_sensors_not_fused or (np.amax(control_mode['using_gps']) > 0.5)
        or (np.amax(control_mode['using_evpos']) > 0.5)):
        sensor_checks.append('pos')
        innov_fail_checks.append('posh')

    # Airspeed Sensor Checks
    # a value > 1.0 means the measurement data for that test has been rejected by the EKF
    if (np.amax(estimator_status['tas_test_ratio']) > 0.0):
        sensor_checks.append('tas')
        innov_fail_checks.append('tas')

    # Height above ground (rangefinder) sensor checks
    if (np.amax(estimator_status['hagl_test_ratio']) > 0.0):
        sensor_checks.append('hagl')
        innov_fail_checks.append('hagl')

    # optical flow sensor checks
    if (np.amax(control_mode['using_optflow']) > 0.5):
        innov_fail_checks.append('ofx')
        innov_fail_checks.append('ofy')

    return sensor_checks, innov_fail_checks


def get_output_tracking_error_message(ulog: ULog) -> str:
    """
    return the name of the message containing the output_tracking_error
    :param ulog:
    :return: str
    """
    msg_name_list = []
    for elem in  ulog.data_list:
        msg_name_list.append(elem.name)
    if "ekf2_innovations" in msg_name_list:
        return "ekf2_innovations"
    elif "estimator_innovations" in msg_name_list:
        return "estimator_status"
    else:
        raise(PreconditionError("Could not detect the message containing the output tracking error"))

def get_innovation_message(ulog: ULog) -> str:
    """
    return the name of the innovation message (old: ekf2_innovations; new: estimator_innovations)
    :param ulog:
    :return: str
    """
    msg_name_list = []
    for elem in  ulog.data_list:
        msg_name_list.append(elem.name)
    if "ekf2_innovations" in msg_name_list:
        return "ekf2_innovations"
    elif "estimator_innovations" in msg_name_list:
        return "estimator_innovations"
    else:
        raise(PreconditionError("Could not detect any known innovation message"))

def get_innovation_variance_message(ulog: ULog) -> str:
    """
    return the name of the innovation variance message
    :param ulog:
    :return: str
    """
    msg_name_list = []
    for elem in  ulog.data_list:
        msg_name_list.append(elem.name)
    if "ekf2_innovations" in msg_name_list:
        return "ekf2_innovations"
    elif "estimator_innovation_variances" in msg_name_list:
        return "estimator_innovation_variances"
    else:
        raise(PreconditionError("Could not detect any known innovation message"))

def get_field_name_from_message_and_descriptor(message: str, field_descriptor: str) -> str:
    """
    return the actual field name (old or new) for a field descriptor
    e.g. (old) message: ekf2_innovations; field_descriptor: magnetometer_innovations -> mag_innov
    :param ulog:
    :return:
    """
    msg_lookUp_dict = {
        'ekf2_innovations' :
            {
                'magnetometer_innovation' : 'mag_innov',
                'heading_innovation' : 'heading_innov',
                'airspeed_innovation' : 'airspeed_innov',
                'sideslip_innovation' : 'beta_innov',
                'flow_innovation' : 'flow_innov',
                'hagl_innovation' : 'hagl_innov',
                'drag_innovation' : 'drag_innov',
                'magnetometer_innovation_variance' : 'mag_innov_var',
                'heading_innovation_variance' : 'heading_innov_var',
                'airspeed_innovation_variance' : 'airspeed_innov_var',
                'sideslip_innovation_variance' : 'beta_innov_var',
                'flow_innovation_variance' : 'flow_innov_var',
                'hagl_innovation_variance' : 'hagl_innov_var',
                'drag_innovation_variance' : 'drag_innov_var'
            },
        'estimator_innovations' :
            {
                'magnetometer_innovation' : 'mag',
                'heading_innovation' : 'heading',
                'airspeed_innovation' : 'airspeed',
                'sideslip_innovation' : 'beta',
                'flow_innovation' : 'flow',
                'hagl_innovation' : 'hagl',
                'drag_innovation' : 'drag',
            },
        'estimator_innovation_variances' :
            {
                'magnetometer_innovation_variance' : 'mag',
                'heading_innovation_variance' : 'heading',
                'airspeed_innovation_variance' : 'airspeed',
                'sideslip_innovation_variance' : 'beta',
                'flow_innovation_variance' : 'flow',
                'hagl_innovation_variance' : 'hagl',
                'drag_innovation_variance' : 'drag'
            }
    }
    field = msg_lookUp_dict.get(message).get(field_descriptor)
    if field == None:
        raise(PreconditionError("Could not find field in lookup table"))
    else:
        return field





