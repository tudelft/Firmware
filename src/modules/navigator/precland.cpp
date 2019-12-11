/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file precland.cpp
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "precland.h"
#include "navigator.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>


//#include <iostream>

#define SEC2USEC 1000000.0f

PrecLand::PrecLand(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
PrecLand::on_activation()
{
	// We need to subscribe here and not in the constructor because constructor is called before the navigator task is spawned
	if (_target_pose_sub < 0) {
		_target_pose_sub = orb_subscribe(ORB_ID(landing_target_pose));
	}
	if (_attitudeSub < 0) {
		_attitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	}

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (!map_projection_initialized(&_map_ref)) {
		map_projection_init(&_map_ref, vehicle_local_position->ref_lat, vehicle_local_position->ref_lon);
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->next.valid = false;
	if (!pos_sp_triplet->current.valid) {
		mavlink_log_info(&mavlink_log_pub,"Resetting search position to current position");

		pos_sp_triplet->next.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->next.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->next.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->next.yaw = _navigator->get_global_position()->yaw;
		pos_sp_triplet->next.yaw_valid = true;
		pos_sp_triplet->next.yawspeed = 0;
		pos_sp_triplet->next.yawspeed_valid = true;
		pos_sp_triplet->next.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		pos_sp_triplet->next.position_valid = true;
		pos_sp_triplet->next.vx = 0;
		pos_sp_triplet->next.vy = 0;
		pos_sp_triplet->next.vz = 0;
		pos_sp_triplet->next.velocity_valid = false;
		pos_sp_triplet->next.alt_valid = true;
		pos_sp_triplet->next.valid = true;


		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.yaw = _navigator->get_global_position()->yaw;
		pos_sp_triplet->current.yaw_valid = true;
		pos_sp_triplet->current.yawspeed = 0;
		pos_sp_triplet->current.yawspeed_valid = true;
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		pos_sp_triplet->current.position_valid = true;
		pos_sp_triplet->current.vx = 0;
		pos_sp_triplet->current.vy = 0;
		pos_sp_triplet->current.vz = 0;
		pos_sp_triplet->current.velocity_valid = false;
		pos_sp_triplet->current.alt_valid = true;
		pos_sp_triplet->current.valid = true;

		pos_sp_triplet->previous.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->previous.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->previous.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->previous.yaw = _navigator->get_global_position()->yaw;
		pos_sp_triplet->previous.yaw_valid = true;
		pos_sp_triplet->previous.yawspeed = 0;
		pos_sp_triplet->previous.yawspeed_valid = true;
		pos_sp_triplet->previous.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		pos_sp_triplet->previous.position_valid = true;
		pos_sp_triplet->previous.vx = 0;
		pos_sp_triplet->previous.vy = 0;
		pos_sp_triplet->previous.vz = 0;
		pos_sp_triplet->previous.velocity_valid = false;
		pos_sp_triplet->previous.alt_valid = true;
		pos_sp_triplet->previous.valid = true;
		_navigator->set_position_setpoint_triplet_updated();
	} else {
		mavlink_log_info(&mavlink_log_pub,"Error, current position not valid");
	}

	_state = PrecLandState::InitSearch;
}

void
PrecLand::on_active()
{
	// get new target measurement
	orb_check(_target_pose_sub, &_target_pose_updated);
	if (_target_pose_updated) {
		orb_copy(ORB_ID(landing_target_pose), _target_pose_sub, &_target_pose);
		_target_pose_initialised = true;
	}
	orb_check(_attitudeSub, &_v_att_updated);
	if (_v_att_updated)
		orb_copy(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);

	if (_target_pose_initialised) {
		if (_target_pose.detected) {
			time_last_sighting = hrt_absolute_time();
		}
		time_since_last_sighting = (hrt_absolute_time() - time_last_sighting);
		time_since_last_sighting /= SEC2USEC;
	} else
		time_since_last_sighting = 2* _param_target_really_lost_timeout.get();

	// stop if we are landed
	if (_navigator->get_land_detected()->landed) {
		_state = PrecLandState::Done;
	}

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	float height = vehicle_local_position->dist_bottom;
	if (height>5 && !_param_marker_distance_type.get()) {
		height = -vehicle_local_position->z;
		if (height<5)
			height=5;
	}

	switch (_state) {
	case PrecLandState::InitSearch: {
		mavlink_log_info(&mavlink_log_pub,"Climbing to search altitude.");
		init_search_triplet();
		mavlink_log_info(&mavlink_log_pub, "Search alt: %.2f", (double)_param_search_alt.get());
		_state = PrecLandState::WaitForTarget;
		// fallthrough
	} case PrecLandState::WaitForTarget: {
		// check if we can see the target, and have a valid estimates
		if (!(debug_msg_div % 12) ) {
			mavlink_log_info(&mavlink_log_pub, "Searching d2b: %.2f h: %.2f alt: %.2f ref: %.2f", (double)vehicle_local_position->dist_bottom, (double) height,(double)_navigator->get_global_position()->alt,(double)vehicle_local_position->ref_alt);
		}
		if ( _target_pose_initialised && _target_pose.detected) {
			_state = PrecLandState::InitApproach;
			// fallthrough
		} else
			break;
	} case PrecLandState::InitApproach: {
		marker_size_mem=0;
		mavlink_log_info(&mavlink_log_pub,"Target found. Approaching.");
		_state = PrecLandState::RunApproach;
		// fallthrough
	} case PrecLandState::RunApproach: {
		// check if target visible
		if (time_since_last_sighting < _param_target_lost_timeout.get()) {
			update_approach(height);
			break;
		} else
			_state = PrecLandState::InitLost;

		break;
	} case PrecLandState::InitLost: {
		mavlink_log_info(&mavlink_log_pub,"Target lost.");
		_state = PrecLandState::RunLost;
		// fallthrough
	} case PrecLandState::RunLost: {
		if ( _target_pose.detected) {
			_state = PrecLandState::InitApproach;
		} else {
			//immidiately increase the area we are looking at
			if (height < _param_search_alt.get() - 2 ) {
				land_speed_smthr.addSample(-_param_pld_v_up_lst.get());
			} else
				land_speed_smthr.addSample(0);

			if (marker_size_mem > _param_final_approach_size.get()*1.5f)
				land_speed_smthr.addSample(0.1f*_param_pld_v_lnd.get()); // final approach ignore lost just descend

			if (no_v_diff_cnt < _param_v_diff_cnt_tresh.get() && height >_param_search_alt.get()*0.75f) {
				//assume that we've lost the target because we weren't moving fast enough
				pos_sp_triplet->current.vx = vx_smthr.get_latest()*2;
				pos_sp_triplet->current.vy = vy_smthr.get_latest()*2;
			}

			if (height>_param_search_alt.get()-2 || _param_only_flw.get()) {
				pos_sp_triplet->current.vz = 0;
				pos_sp_triplet->current.alt_valid = true;
				pos_sp_triplet->current.alt = _param_search_alt.get()+ vehicle_local_position->ref_alt;
			} else {
				pos_sp_triplet->current.vz = land_speed_smthr.get_latest();
				pos_sp_triplet->current.alt_valid = false;
			}

			pos_sp_triplet->current.velocity_valid = true;
			pos_sp_triplet->current.position_valid = false;
			pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
			pos_sp_triplet->next.valid = false;

			_navigator->set_position_setpoint_triplet_updated();

			if (!(debug_msg_div % 12) ) {
				mavlink_log_info(&mavlink_log_pub, "Lost lvz: %.2f d2b: %.2f h: %.2f", (double)land_speed_smthr.get_latest(), (double)vehicle_local_position->dist_bottom, (double) height);
			}

		}

		if (time_since_last_sighting > _param_target_really_lost_timeout.get())
			_state = PrecLandState::InitSearch;

		break;
	} case PrecLandState::Done: {
		marker_size_mem = 0;
		if (!(debug_msg_div % 12))
			mavlink_log_info(&mavlink_log_pub,"Landing done!");
		break;
	} default:
		break;
	}

	debug_msg_div++; // only for printing debug info
}

void PrecLand::init_search_triplet() {
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.alt = _param_search_alt.get()+vehicle_local_position->ref_alt;
	pos_sp_triplet->current.alt_valid = true;
	pos_sp_triplet->current.velocity_valid = false;
	pos_sp_triplet->current.position_valid = true;
	pos_sp_triplet->current.vx = 0;
	pos_sp_triplet->current.vy = 0;
	pos_sp_triplet->current.vz = 0;
	pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
	pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
	pos_sp_triplet->current.valid = true;
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	vx_smthr.init();
	vy_smthr.init();
	land_speed_smthr.init();
	angle_x_i_err = 0;
	angle_y_i_err = 0;
	no_v_diff_cnt =0;
	_target_pose_initialised = false;
	_target_pose_updated = false;
	_navigator->set_position_setpoint_triplet_updated();
}

void PrecLand::update_approach_land_speed(float height) {

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
	if (_target_pose.detected && _target_pose.marker_size > 0)
		marker_size_mem  = _target_pose.marker_size;

	if (!_target_pose.detected && marker_size_mem>_param_final_approach_size.get()) {
		land_speed_smthr.addSample(0.1f*_param_pld_v_lnd.get());
	} else if(in_acceptance_range() && time_since_last_sighting < 1.f ) { // todo use param
		float a = sqrtf(powf(_target_pose.angle_x,2)+powf(_target_pose.angle_y,2));
		float a_max = _param_hacc_rad.get(); //this is camera (fov) dependent
		//is in the range of approx [0 - a_max], when it is 0 we want to descend as fast a possible.
		//When it's a_max stop descending.

		float max_land_speed = _param_pld_v_lnd.get();
		if  (marker_size_mem<_param_final_approach_size.get()*0.75f)
			max_land_speed*=2.f;
		else if  (marker_size_mem<_param_final_approach_size.get()*0.5f)
			max_land_speed*=4.f;
		else {
			max_land_speed*=(_param_final_approach_size.get() / _target_pose.marker_size);
			if (max_land_speed<0.1f)
				max_land_speed = 0.1;
			else if (max_land_speed>1)
				max_land_speed= 1.f;
		}

		float land_speed;
		if (a<_param_hacc_rad.get()/4.f)
			land_speed = max_land_speed;
		else if (a>_param_hacc_rad.get())
			land_speed = 0.01f;
		else
			land_speed = max_land_speed * (1.f/a_max) *(a_max-a);

//        if  (_target_pose.marker_size>_param_final_approach_size.get() && a < 0.1f)
//            land_speed  = 0.1f*_param_pld_v_lnd.get();

		if (_param_only_flw.get())
			land_speed = 0;
		if (land_speed<0)
			land_speed = 0;

		land_speed_smthr.addSample(land_speed*0.1f + 0.9f*land_speed_smthr.get_latest());
		if (!(debug_msg_div % 12)) {
			if  (_target_pose.marker_size>_param_final_approach_size.get() && a < 0.1f) {
				mavlink_log_info(&mavlink_log_pub, "Final approach  a %.2f lvz: %.2f d2b: %.2f h: %.2f size %.2f", (double)a, (double)land_speed_smthr.get_latest(),(double)vehicle_local_position->dist_bottom,(double)height,(double)marker_size_mem );
			} else {
				mavlink_log_info(&mavlink_log_pub, "Descending  a %.2f lvz: %.2f d2b: %.2f h: %.2f size %.2f", (double)a, (double)land_speed_smthr.get_latest(),(double)vehicle_local_position->dist_bottom,(double)height,(double)marker_size_mem );
			}
		}
	} else {
		if (!(debug_msg_div % 12) ) {
			bool pos_control_enabled = no_v_diff_cnt < _param_v_diff_cnt_tresh.get()+2;
			float a = sqrtf(powf(fabs(_target_pose.angle_x),2)+powf(fabs(_target_pose.angle_y),2));
			if (pos_control_enabled) {
				mavlink_log_info(&mavlink_log_pub, "Catching up a %.2f lvz: %.2f d2b: %.2f h: %.2f size %.2f", (double)a, (double)land_speed_smthr.get_latest(),(double)vehicle_local_position->dist_bottom,(double)height,(double)marker_size_mem );
			} else {
				mavlink_log_info(&mavlink_log_pub, "Positioning a %.2f lvz: %.2f d2b: %.2f h: %.2f size %.2f", (double)a, (double)land_speed_smthr.get_latest(), (double)vehicle_local_position->dist_bottom,(double)height,(double)marker_size_mem );
			}
		}

		land_speed_smthr.addSample(0.f);
	}
}

void PrecLand::update_approach(float height) {

	update_approach_land_speed(height);

	if (!_target_pose.detected)
		return;

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	double lat, lon;
	map_projection_reproject(&_map_ref, _target_pose.x_abs,_target_pose.y_abs, &lat, &lon);

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;
	if (no_v_diff_cnt < _param_v_diff_cnt_tresh.get()) {
		// reset integrator error when marker v estimation is still stabilizing
		angle_x_i_err = 0;
		angle_y_i_err = 0;
	}

	if(height > 10 && _target_pose.abs_pos_valid) { //assume no sudden changes in marker speed are happening when the drone is in low landing
		if(no_v_diff_cnt > _param_v_diff_cnt_tresh.get()) {//when v is matched, assume the ship doesn't make sudden changes in speed
			//this acts as a sort of I gain
			pos_sp_triplet->current.vx = vx_smthr.addSample(vx_smthr.get_latest()*0.9f + _target_pose.vx_abs*0.1f);
			pos_sp_triplet->current.vy = vy_smthr.addSample(vy_smthr.get_latest()*0.9f + _target_pose.vy_abs*0.1f);
		} else {
			pos_sp_triplet->current.vx = vx_smthr.addSample(_target_pose.vx_abs);
			pos_sp_triplet->current.vy = vy_smthr.addSample(_target_pose.vy_abs);
		}
	}
	pos_sp_triplet->current.vx = vx_smthr.get_latest();
	pos_sp_triplet->current.vy = vy_smthr.get_latest();

	float ss_p_gain = _param_pld_xy_g_p.get(); //pos p gain
	float ss_i_gain = _param_pld_xy_g_i.get()/100.f; // pos i gai
	float ss_d_gain = _param_pld_xy_g_d.get(); //pos p gain
	float ss_d_drot_gain = _param_pld_xy_g_dd.get();

	//only activate pos control when drone is up to speed
	float dv = sqrtf(powf(_target_pose.vx_rel,2) + powf(_target_pose.vy_rel,2));

	matrix::Eulerf euler = matrix::Quatf(_vehicleAttitude.q);
	float body_angle_x = cosf(euler.psi()) * _target_pose.angle_x - sinf(euler.psi()) * _target_pose.angle_y;
	float body_angle_y = sinf(euler.psi()) * _target_pose.angle_x + cosf(euler.psi()) * _target_pose.angle_y;
	static float body_angle_x_prev = body_angle_x;
	static float body_angle_y_prev = body_angle_y;
	float d_angle_x_err =body_angle_x - body_angle_x_prev;
	float d_angle_y_err =body_angle_y - body_angle_y_prev;
	body_angle_x_prev = body_angle_x;
	body_angle_y_prev = body_angle_y;

	float drot_angle_x = body_angle_x - euler.phi();
	float drot_angle_y = body_angle_y - euler.theta();
	static float drot_angle_x_prev = drot_angle_x;
	static float drot_angle_y_prev = drot_angle_y;
	static float phi_prev = euler.phi();
	static float theta_prev = euler.theta();
	float d_drot_x =(drot_angle_x_prev - drot_angle_x) - (phi_prev - euler.phi());
	float d_drot_y =(drot_angle_y_prev - drot_angle_y) - (theta_prev - euler.theta());
	drot_angle_x_prev = drot_angle_x;
	drot_angle_y_prev = drot_angle_y;
	phi_prev = euler.phi();
	theta_prev = euler.theta();

	int tresh = _param_v_diff_cnt_tresh.get();

	if (dv<1 && no_v_diff_cnt < tresh+2 && _target_pose.abs_pos_valid && _target_pose_updated && vx_smthr.ready())
		no_v_diff_cnt++;
	if (no_v_diff_cnt > tresh) {
		angle_x_i_err+=_target_pose.angle_x;
		angle_y_i_err+=_target_pose.angle_y;

		//scale p&i gain to oscilations:
		float fx = 1.f/_target_pose.movvar_x;
		float fy = 1.f/_target_pose.movvar_y;

		if (isnan(fx))
			fx = 1.f;
		if (fx < 0.01f)
			fx = 0.01f;
		if (fx > 40.f)
			fx = 40.f;
		if (isnan(fy))
			fy = 1.f;
		if (fy < 0.01f)
			fy = 0.01f;
		if (fy > 40.f)
			fy = 40.f;

		if (_target_pose.marker_size < 100 && _target_pose.marker_size >0) {
			fx/=2.f;
			fy/=2.f;
		}

		if (!_param_marker_use_movvar.get()) {
			fx = 1; //TMP science test  use p = 7, d = 12 to be on edge of osc
			fy = 1; //TMP science test
		}

		float ss_p_gain_x =ss_p_gain*fx;
		float ss_p_gain_y =ss_p_gain*fy;
		float ss_d_gain_x =ss_d_gain; //*fx;
		float ss_d_gain_y =ss_d_gain; //*fy;
		float ss_drot_gain_x =ss_d_drot_gain;
		float ss_drot_gain_y =ss_d_drot_gain;

		float ss_i_gain_x = ss_i_gain*fx;
		float ss_i_gain_y = ss_i_gain*fy;

		float ss_vx = ss_p_gain_x * _target_pose.angle_x + angle_x_i_err * ss_i_gain_x;
		float ss_vy = ss_p_gain_y * _target_pose.angle_y + angle_y_i_err * ss_i_gain_y;

		//rotate and add P & I
		pos_sp_triplet->current.vx += cosf(euler.psi()) * ss_vx - sinf(euler.psi()) * ss_vy;
		pos_sp_triplet->current.vy += sinf(euler.psi()) * ss_vx + cosf(euler.psi()) * ss_vy;

		//D was already rotated
		pos_sp_triplet->current.vx += ss_d_gain_x*d_angle_x_err + ss_drot_gain_x*d_drot_x;
		pos_sp_triplet->current.vy += ss_d_gain_y*d_angle_y_err + ss_drot_gain_y*d_drot_y;

	}

	if (!_param_only_flw.get()) {
		pos_sp_triplet->current.vz = land_speed_smthr.get_latest();
		pos_sp_triplet->current.alt_valid = false;
	} else {
		pos_sp_triplet->current.vz = 0;
		pos_sp_triplet->current.alt = _param_search_alt.get() + vehicle_local_position->ref_alt;
		pos_sp_triplet->current.alt_valid = true;
	}

	pos_sp_triplet->current.velocity_valid = true;
	pos_sp_triplet->current.position_valid = false;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->current.yawspeed_valid = false;
	_navigator->set_position_setpoint_triplet_updated();

//    std::cout << _target_pose.timestamp / 1000000.f << "; "
//          << euler.phi() << "; " << euler.theta() << "; "
//          << _target_pose.angle_x << "; " << _target_pose.angle_y << "; "
//          << _target_pose.movvar_x << "; "  << _target_pose.movvar_y << "; "
//          << _target_pose.vx_abs << "; " << _target_pose.vy_abs << "; "
//          << _target_pose.vx_rel << "; " << _target_pose.vy_rel << "; "
//          << body_angle_x << "; " << body_angle_y << "; "
//          << drot_angle_x << "; " << drot_angle_y << "; "
//          << pos_sp_triplet->current.vx << "; " << pos_sp_triplet->current.vy << "; " << std::endl;
}

bool PrecLand::in_acceptance_range() {
	if (_target_pose.detected) {
		float a = sqrtf(powf(_target_pose.angle_x,2)+powf(_target_pose.angle_y,2));
		return (a<_param_hacc_rad.get() && no_v_diff_cnt>=_param_v_diff_cnt_tresh.get() );
	} else {
		return false;
	}
}













//TODO: move to seperate file
void Smoother_10::init(float value)
{
	_rotater = 0;
	_runner = value*_kernelsize;
	for (int i = 0; i < _kernelsize; i++)
	{
		_rbuf(i) = value;
	}
	_ready = true;
}

void Smoother_10::init()
{
	init(0);
	_ready = false;
}

void Smoother_10::reset()
{
	_rotater = 0;
	for (int i = 0; i <= _kernelsize; i++)
	{
		_rbuf(i) = 0;
	}
	_runner = 0;
	_ready = false;
}
void Smoother_10::reset_to(float v)
{
	_rotater = 0;
	for (int i = 0; i <= _kernelsize; i++)
	{
		_rbuf(i) = v;
	}
	_runner = v*_kernelsize;
	_ready = true;
}

float Smoother_10::addSample(float sample)
{
	if (isnan(sample)) // fixes nan, which forever destroy the output
		sample = 0;
	if (_kernelsize == 1)
	{	// disable smoothing... to be sure:
		_ready = true;
		_runner = sample;
		return sample;
	}

	_rbuf(_rotater) = sample;                     // overwrite oldest sample in the roundtrip buffer
	_rotater = (_rotater + 1) % (_kernelsize);   //update pointer to buffer
	_runner = _runner + sample - _rbuf(_rotater); //add new sample, subtract the new oldest sample

	if (!_ready)
	{	// check if completely filled
		if (_rotater == 0)
			_ready = true;
		else
			return _runner / _rotater; // if not filled completely, return average over the amount of added data (the rest of the filter is initialised to zero)
	}

	return _runner / (_kernelsize-1);
}

float Smoother_10::get_latest()
{
	return _runner / (_kernelsize-1);
}
//lalala

void Smoother_100::init(float value)
{
	_rotater = 0;
	_runner = value*_kernelsize;
	for (int i = 0; i < _kernelsize; i++)
	{
		_rbuf(i) = value;
	}
	_ready = true;
}

void Smoother_100::init()
{
	init(0);
	_ready = false;
}

void Smoother_100::reset()
{
	_rotater = 0;
	for (int i = 0; i <= _kernelsize; i++)
	{
		_rbuf(i) = 0;
	}
	_runner = 0;
	_ready = false;
}
void Smoother_100::reset_to(float v)
{
	_rotater = 0;
	for (int i = 0; i <= _kernelsize; i++)
	{
		_rbuf(i) = v;
	}
	_runner = v*_kernelsize;
	_ready = true;
}

float Smoother_100::addSample(float sample)
{
	if (isnan(sample)) // fixes nan, which forever destroy the output
		sample = 0;
	if (_kernelsize == 1)
	{	// disable smoothing... to be sure:
		_ready = true;
		_runner = sample;
		return sample;
	}

	_rbuf(_rotater) = sample;                     // overwrite oldest sample in the roundtrip buffer
	_rotater = (_rotater + 1) % (_kernelsize);   //update pointer to buffer
	_runner = _runner + sample - _rbuf(_rotater); //add new sample, subtract the new oldest sample

	if (!_ready)
	{	// check if completely filled
		if (_rotater == 0)
			_ready = true;
		else
			return _runner / _rotater; // if not filled completely, return average over the amount of added data (the rest of the filter is initialised to zero)
	}

	return _runner / (_kernelsize-1);
}

float Smoother_100::get_latest()
{
	return _runner / (_kernelsize-1);
}
