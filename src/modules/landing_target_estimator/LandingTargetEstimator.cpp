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
	_new_irlockReport(false),
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

	if (!_new_irlockReport) {
		// nothing to do
		return;
	}

	// mark this sensor measurement as consumed
	_new_irlockReport = false;

	if (!_vehicleAttitude_valid || !_vehicleLocalPosition_valid || !_vehicleLocalPosition.dist_bottom_valid) {
		// don't have the data needed for an update
		return;
	}

	if (!PX4_ISFINITE(_irlockReport.pos_y) || !PX4_ISFINITE(_irlockReport.pos_x)) {
		return;
	}

	// TODO account for sensor orientation as set by parameter
	// default orientation has camera x pointing in body y, camera y in body -x

	matrix::Vector<float, 3> sensor_ray; // ray pointing towards target in body frame
	sensor_ray(0) = -_irlockReport.pos_y * _params.scale_y; // forward
	sensor_ray(1) = _irlockReport.pos_x * _params.scale_x; // right
	sensor_ray(2) = 1.0f;
	//printf("irlock: %f, %f\n",(double)sensor_ray(0),(double)sensor_ray(1));


	matrix::Vector<float, 3> zero_ray; // to check what would be the projected location of a target if it would be in the middle of the image
	zero_ray(0) = 0.0f;
	zero_ray(1) = 0.0f;
	zero_ray(2) = 1.0f;

	// rotate the unit ray into the navigation frame, assume sensor frame = body frame
	matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
	_R_att = matrix::Dcm<float>(q_att);
	sensor_ray = _R_att * sensor_ray;
	zero_ray = _R_att * zero_ray;

	if (fabsf(sensor_ray(2)) < 1e-6f) {
		// z component of measurement unsafe, don't use this measurement
		return;
	}

	float dist = _vehicleLocalPosition.dist_bottom;

	// scale the ray s.t. the z component has length of dist
	_rel_pos(0) = sensor_ray(0) / sensor_ray(2) * dist;
	_rel_pos(1) = sensor_ray(1) / sensor_ray(2) * dist;

	_zero_rel_pos(0) = zero_ray(0) / zero_ray(2) * dist;
	_zero_rel_pos(1) = zero_ray(1) / zero_ray(2) * dist;

	if (!_estimator_initialized) {
		PX4_INFO("Init");
		float vx_init = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vx : 0.f;
		float vy_init = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vy : 0.f;
		_kalman_filter_x.init(_rel_pos(0), vx_init, _params.pos_unc_init, _params.vel_unc_init);
		_kalman_filter_y.init(_rel_pos(1), vy_init, _params.pos_unc_init, _params.vel_unc_init);

		_kalman_filter_zero_x.init(_zero_rel_pos(0), vx_init, _params.pos_unc_init, _params.vel_unc_init);
		_kalman_filter_zero_y.init(_zero_rel_pos(1), vy_init, _params.pos_unc_init, _params.vel_unc_init);

		_estimator_initialized = true;
		_last_update = hrt_absolute_time();
		_last_predict = _last_update;

	} else {
		// update
		bool update_x = _kalman_filter_x.update(_rel_pos(0), _params.meas_unc * dist * dist);
		bool update_y = _kalman_filter_y.update(_rel_pos(1), _params.meas_unc * dist * dist);

		bool zero_update_x = _kalman_filter_zero_x.update(_zero_rel_pos(0), _params.meas_unc * dist * dist);
		bool zero_update_y = _kalman_filter_zero_y.update(_zero_rel_pos(1), _params.meas_unc * dist * dist);

		if (!update_x || !update_y) {
			if (!_faulty) {
				_faulty = true;
				PX4_WARN("Landing target measurement rejected:%s%s", update_x ? "" : " x", update_y ? "" : " y");
			}

		} else {
			_faulty = false;
		}
		if (!update_x || !update_y) {
			if (!_zero_faulty) {
				_zero_faulty=true;
				PX4_WARN("Landing target zero measurement rejected:%s%s", zero_update_x ? "" : " x", zero_update_y ? "" : " y");
			}
		} else {
			_zero_faulty = false;
		}

		if (!_faulty) {
			// only publish if both measurements were good

			_target_pose.timestamp = _irlockReport.timestamp;

			float x, xvel, y, yvel, covx, covx_v, covy, covy_v;
			float zero_x, zero_xvel, zero_y, zero_yvel, zero_covx, zero_covx_v, zero_covy, zero_covy_v;
			_kalman_filter_x.getState(x, xvel);
			_kalman_filter_x.getCovariance(covx, covx_v);

			_kalman_filter_y.getState(y, yvel);
			_kalman_filter_y.getCovariance(covy, covy_v);

			_kalman_filter_zero_x.getState(zero_x, zero_xvel);
			_kalman_filter_zero_x.getCovariance(zero_covx, zero_covx_v);

			_kalman_filter_zero_y.getState(zero_y, zero_yvel);
			_kalman_filter_zero_y.getCovariance(zero_covy, zero_covy_v);

			_target_pose.is_static = (_params.mode == TargetMode::Stationary);

			_target_pose.rel_pos_valid = !_faulty;
			_target_pose.rel_vel_valid = !_faulty;
			_target_pose.x_rel = x;
			_target_pose.y_rel = y;
			_target_pose.z_rel = dist;
			_target_pose.vx_rel = xvel;
			_target_pose.vy_rel = yvel;
			_target_pose.cov_x_rel = covx;
			_target_pose.cov_y_rel = covy;
			_target_pose.cov_vx_rel = covx_v;
			_target_pose.cov_vy_rel = covy_v;

			_target_pose.zero_rel_pos_valid = !_zero_faulty;
			_target_pose.zero_rel_vel_valid = !_zero_faulty;
			_target_pose.zero_x_rel = zero_x;
			_target_pose.zero_y_rel = zero_y;
			_target_pose.zero_z_rel = dist;
			_target_pose.zero_vx_rel = zero_xvel;
			_target_pose.zero_vy_rel = zero_yvel;
			_target_pose.zero_cov_x_rel = zero_covx;
			_target_pose.zero_cov_y_rel = zero_covy;
			_target_pose.zero_cov_vx_rel = covx_v;
			_target_pose.zero_cov_vy_rel = covy_v;


			if (_vehicleLocalPosition_valid && _vehicleLocalPosition.xy_valid) {
				_target_pose.x_abs = x + _vehicleLocalPosition.x;
				_target_pose.y_abs = y + _vehicleLocalPosition.y;
				_target_pose.z_abs = dist + _vehicleLocalPosition.z;
				_target_pose.abs_pos_valid = true;

				_target_pose.zero_x_abs = zero_x + _vehicleLocalPosition.x;
				_target_pose.zero_y_abs = zero_y + _vehicleLocalPosition.y;
				_target_pose.zero_z_abs = dist + _vehicleLocalPosition.z;
				_target_pose.zero_abs_pos_valid = true;

			} else {
				_target_pose.abs_pos_valid = false;
				_target_pose.zero_abs_pos_valid = false;
			}

			float a;
			if (fabs(sensor_ray(0)) > fabs(sensor_ray(1)))
				a = fabs(sensor_ray(0));
			else
				a = fabs(sensor_ray(1));
			if (a>0.4f) // 30 degrees, cause realsense FOV. Todo: width FOV is bigger, need to split
				a = 0.5f;
			a = a / 0.5f; // normalize between 0 - 1;
			_target_pose.raw_angle = 1.f-a;

			if (_targetPosePub == nullptr) {
				_targetPosePub = orb_advertise(ORB_ID(landing_target_pose), &_target_pose);

			} else {
				orb_publish(ORB_ID(landing_target_pose), _targetPosePub, &_target_pose);
			}

			PX4_INFO("%f, %f vs %f. %f",(double)_target_pose.x_rel,(double)_target_pose.y_rel,(double)_target_pose.zero_x_rel,(double)_target_pose.zero_y_rel );

			_last_update = hrt_absolute_time();
			_last_predict = _last_update;
		}

		float innov_x, innov_cov_x, innov_y, innov_cov_y;
		_kalman_filter_x.getInnovations(innov_x, innov_cov_x);
		_kalman_filter_y.getInnovations(innov_y, innov_cov_y);

		_target_innovations.timestamp = _irlockReport.timestamp;
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
	_irlockReportSub = orb_subscribe(ORB_ID(irlock_report));
	_parameterSub = orb_subscribe(ORB_ID(parameter_update));
}

void LandingTargetEstimator::_update_topics()
{
	_vehicleLocalPosition_valid = _orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub,
						  &_vehicleLocalPosition);
	_vehicleAttitude_valid = _orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);
	_sensorBias_valid = _orb_update(ORB_ID(sensor_bias), _sensorBiasSub, &_sensorBias);

	_new_irlockReport = _orb_update(ORB_ID(irlock_report), _irlockReportSub, &_irlockReport);
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
