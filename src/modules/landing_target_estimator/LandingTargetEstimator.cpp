/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/*
 * @file LandingTargetEstimator.cpp
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 * @author Mohammed Kabir <kabir@uasys.io>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>

#include "LandingTargetEstimator.h"

#define SEC2USEC 1000000.0f


namespace landing_target_estimator
{

LandingTargetEstimator::LandingTargetEstimator() :
	_targetPosePub(nullptr),
	_targetInnovationsPub(nullptr),
	_paramHandle(),
	_vehicleLocalPosition_valid(false),
	_vehicleAttitude_valid(false),
	_sensorBias_valid(false),
	_new_moving_marker_report(false),
	_estimator_initialized(false),
	_faulty(false),
	_last_predict(0),
	_last_update(0)
{
	_paramHandle.acc_unc = param_find("LTEST_ACC_UNC");
	_paramHandle.meas_unc = param_find("LTEST_MEAS_UNC");
	_paramHandle.pos_unc_init = param_find("LTEST_POS_UNC_IN");
	_paramHandle.vel_unc_init = param_find("LTEST_VEL_UNC_IN");
	_paramHandle.mode = param_find("LTEST_MODE");
	_paramHandle.scale_x = param_find("LTEST_SCALE_X");
	_paramHandle.scale_y = param_find("LTEST_SCALE_Y");

	// Initialize uORB topics.
	_initialize_topics();

	_check_params(true);
}

LandingTargetEstimator::~LandingTargetEstimator() = default;

void LandingTargetEstimator::update()
{
	_check_params(false);

	_update_topics();

	/* predict */
	if (_estimator_initialized) {
		if (hrt_absolute_time() - _last_update > landing_target_estimator_TIMEOUT_US) {
			PX4_WARN("Timeout");
			_estimator_initialized = false;

		} else {
			float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;

			// predict target position with the help of accel data
			matrix::Vector3f a;

			if (_vehicleAttitude_valid && _sensorBias_valid) {
				matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
				_R_att = matrix::Dcm<float>(q_att);
				a(0) = _sensorBias.accel_x;
				a(1) = _sensorBias.accel_y;
				a(2) = _sensorBias.accel_z;
				a = _R_att * a;

			} else {
				a.zero();
			}

			_kalman_filter_x.predict(dt, -a(0), _params.acc_unc);
			_kalman_filter_y.predict(dt, -a(1), _params.acc_unc);

			_kalman_filter_zero_x.predict(dt, -a(0), _params.acc_unc);
			_kalman_filter_zero_y.predict(dt, -a(1), _params.acc_unc);

			_last_predict = hrt_absolute_time();
		}
	}

	if (!_new_moving_marker_report) {
		return;
	}

	// mark this sensor measurement as consumed
	_new_moving_marker_report = false;

	if (!_vehicleAttitude_valid || !_vehicleLocalPosition_valid) {
		// don't have the data needed for an update
		return;
	}

	if (!PX4_ISFINITE(_moving_marker_report.angle_x) || !PX4_ISFINITE(_moving_marker_report.angle_y)
			|| _moving_marker_report.size < 0) {
		_target_pose.detected = false;
		return;
	}

	// TODO account for sensor orientation as set by parameter
	// default orientation has camera x pointing in body y, camera y in body -x


	float dist = _vehicleLocalPosition.dist_bottom;

	if (dist > 5) {
		dist = -_vehicleLocalPosition.z;

		if (dist < 5) {
			dist = 5;
		}
	}

	float alpha = sqrtf(powf(_moving_marker_report.angle_y, 2) + powf(_moving_marker_report.angle_x, 2));
	float dist_to_marker = dist / cosf(alpha);

	// scale the ray such that the z component has length of dist
	_rel_pos(0) = sinf(_moving_marker_report.angle_y) * dist_to_marker;
	_rel_pos(1) = sinf(_moving_marker_report.angle_x) * dist_to_marker;

	float x_abs = _rel_pos(0) + _vehicleLocalPosition.x;
	float y_abs = _rel_pos(1) + _vehicleLocalPosition.y;
	float zero_x_abs = _zero_rel_pos(0) + _vehicleLocalPosition.x;
	float zero_y_abs = _zero_rel_pos(1) + _vehicleLocalPosition.y;

	if (!_estimator_initialized) {
		PX4_INFO("Init");
		_kalman_filter_x.init(x_abs, 0, _params.pos_unc_init, _params.vel_unc_init);
		_kalman_filter_y.init(y_abs, 0, _params.pos_unc_init, _params.vel_unc_init);

		_kalman_filter_zero_x.init(zero_x_abs, 0, _params.pos_unc_init, _params.vel_unc_init);
		_kalman_filter_zero_y.init(zero_y_abs, 0, _params.pos_unc_init, _params.vel_unc_init);

		_estimator_initialized = true;
		_last_update = hrt_absolute_time();
		_last_predict = _last_update;

	} else {
		// update
		bool update_x = _kalman_filter_x.update(x_abs, _params.meas_unc * dist * dist);
		bool update_y = _kalman_filter_y.update(y_abs, _params.meas_unc * dist * dist);

		bool zero_update_x = _kalman_filter_zero_x.update(zero_x_abs, _params.meas_unc * dist * dist);
		bool zero_update_y = _kalman_filter_zero_y.update(zero_y_abs, _params.meas_unc * dist * dist);

		if (!update_x || !update_y) {
			if (!_faulty) {
				_faulty = true;
				PX4_WARN("Landing target measurement rejected:%s%s", update_x ? "" : " x", update_y ? "" : " y");
			}

		} else {
			_faulty = false;
		}
		if (!zero_update_x || !zero_update_y) {
			if (!_zero_faulty) {
				_zero_faulty = true;
				PX4_WARN("Landing target zero measurement rejected:%s%s", zero_update_x ? "" : " x", zero_update_y ? "" : " y");
			}
		} else {
			_zero_faulty = false;
		}

		_target_pose.angle_x = _moving_marker_report.angle_y;
		_target_pose.angle_y = _moving_marker_report.angle_x;
		_target_pose.timestamp = _moving_marker_report.timestamp;
		_target_pose.movvar_x = _moving_marker_report.movvar_y;
		_target_pose.movvar_y = _moving_marker_report.movvar_x;
		_target_pose.detected = true;
		_target_pose.marker_size = _moving_marker_report.size;
		_target_pose.marker_distance = _moving_marker_report.distance;

		if (!_faulty) {
			// only add if both measurements were good
			float x, xvel, y, yvel, covx, covx_v, covy, covy_v;
			float zero_x, zero_xvel, zero_y, zero_yvel;
			_kalman_filter_x.getState(x, xvel);
			_kalman_filter_x.getCovariance(covx, covx_v);

			_kalman_filter_y.getState(y, yvel);
			_kalman_filter_y.getCovariance(covy, covy_v);

			_kalman_filter_zero_x.getState(zero_x, zero_xvel);
			_kalman_filter_zero_y.getState(zero_y, zero_yvel);

			_target_pose.is_static = (_params.mode == TargetMode::Stationary);

			_target_pose.rel_pos_valid = !_faulty;
			_target_pose.rel_vel_valid = !_faulty;
			_target_pose.x_rel = x - _vehicleLocalPosition.x;
			_target_pose.y_rel = y - _vehicleLocalPosition.y;
			_target_pose.z_rel = dist;
			if (_vehicleLocalPosition.v_xy_valid) {
				_target_pose.vx_rel = xvel - _vehicleLocalPosition.vx;
				_target_pose.vy_rel = yvel - _vehicleLocalPosition.vy;
			} else {
				_target_pose.vx_rel = 0;
				_target_pose.vy_rel = 0;
			}

			_target_pose.vx_abs = xvel;
			_target_pose.vy_abs = yvel;
			_target_pose.cov_x_rel = covx;
			_target_pose.cov_y_rel = covy;
			_target_pose.cov_vx_rel = covx_v;
			_target_pose.cov_vy_rel = covy_v;


			_target_pose.zero_rel_pos_valid = !_zero_faulty;
			_target_pose.zero_x_rel = zero_x - _vehicleLocalPosition.x;
			_target_pose.zero_y_rel = zero_y - _vehicleLocalPosition.y;

			if (_vehicleLocalPosition_valid && _vehicleLocalPosition.xy_valid) {
				_target_pose.x_abs = x;
				_target_pose.y_abs = y;
				_target_pose.abs_pos_valid = _target_pose.rel_pos_valid;

				_target_pose.zero_x_abs = zero_x;
				_target_pose.zero_y_abs = zero_y;
				_target_pose.zero_abs_pos_valid = _target_pose.rel_pos_valid;

			} else {
				_target_pose.abs_pos_valid = false;
				_target_pose.zero_abs_pos_valid = false;
			}

			_target_pose.raw_angle = -666; // TODO remove

			_last_update = hrt_absolute_time();
			_last_predict = _last_update;
		} else {
			_target_pose.abs_pos_valid = false;
			_target_pose.zero_abs_pos_valid = false;
			_target_pose.rel_pos_valid = false;
			_target_pose.rel_vel_valid = false;
			_target_pose.zero_abs_pos_valid = false;
		}

		if (_targetPosePub == nullptr) {
			_targetPosePub = orb_advertise(ORB_ID(landing_target_pose), &_target_pose);

		} else {
			orb_publish(ORB_ID(landing_target_pose), _targetPosePub, &_target_pose);
		}

		float innov_x, innov_cov_x, innov_y, innov_cov_y;
		_kalman_filter_x.getInnovations(innov_x, innov_cov_x);
		_kalman_filter_y.getInnovations(innov_y, innov_cov_y);

		_target_innovations.timestamp = _moving_marker_report.timestamp;
		_target_innovations.innov_x = innov_x;
		_target_innovations.innov_cov_x = innov_cov_x;
		_target_innovations.innov_y = innov_y;
		_target_innovations.innov_cov_y = innov_cov_y;

		if (_targetInnovationsPub == nullptr) {
			_targetInnovationsPub = orb_advertise(ORB_ID(landing_target_innovations), &_target_innovations);

		} else {
			orb_publish(ORB_ID(landing_target_innovations), _targetInnovationsPub, &_target_innovations);
		}
	}

}

void LandingTargetEstimator::_check_params(const bool force)
{
	bool updated;
	parameter_update_s paramUpdate;

	orb_check(_parameterSub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameterSub, &paramUpdate);
	}

	if (updated || force) {
		_update_params();
	}
}

void LandingTargetEstimator::_initialize_topics()
{
	_vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
	_attitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	_sensorBiasSub = orb_subscribe(ORB_ID(sensor_bias));
	_moving_marker_reportSub = orb_subscribe(ORB_ID(moving_marker_report));
	_parameterSub = orb_subscribe(ORB_ID(parameter_update));
}

void LandingTargetEstimator::_update_topics()
{
	_vehicleLocalPosition_valid = _orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub, &_vehicleLocalPosition);
	_vehicleAttitude_valid = _orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);
	_sensorBias_valid = _orb_update(ORB_ID(sensor_bias), _sensorBiasSub, &_sensorBias);
	_new_moving_marker_report = _orb_update(ORB_ID(moving_marker_report), _moving_marker_reportSub, &_moving_marker_report);
}


bool LandingTargetEstimator::_orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	// check if there is new data to grab
	if (orb_check(handle, &newData) != OK) {
		return false;
	}

	if (!newData) {
		return false;
	}

	if (orb_copy(meta, handle, buffer) != OK) {
		return false;
	}

	return true;
}

void LandingTargetEstimator::_update_params()
{
	param_get(_paramHandle.acc_unc, &_params.acc_unc);
	param_get(_paramHandle.meas_unc, &_params.meas_unc);
	param_get(_paramHandle.pos_unc_init, &_params.pos_unc_init);
	param_get(_paramHandle.vel_unc_init, &_params.vel_unc_init);
	int32_t mode = 0;
	param_get(_paramHandle.mode, &mode);
	_params.mode = (TargetMode)mode;
	param_get(_paramHandle.scale_x, &_params.scale_x);
	param_get(_paramHandle.scale_y, &_params.scale_y);
}


} // namespace landing_target_estimator
