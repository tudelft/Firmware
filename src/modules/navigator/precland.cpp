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

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (!map_projection_initialized(&_map_ref)) {
		map_projection_init(&_map_ref, vehicle_local_position->ref_lat, vehicle_local_position->ref_lon);
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->next.valid = false;
	if (!pos_sp_triplet->current.valid) {
		mavlink_log_info(&mavlink_log_pub,"Resetting search position to current position");
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.valid = true;
		_navigator->set_position_setpoint_triplet_updated();
	} else {
		mavlink_log_info(&mavlink_log_pub,"Error, current position not valid");
	}

	_state = PrecLandState::InitSearch;
}

void
PrecLand::on_active()
{
	debug_msg_div++; // tmp only for printing debug info

	// get new target measurement
	orb_check(_target_pose_sub, &_target_pose_updated);

	if (_target_pose_updated) {
		orb_copy(ORB_ID(landing_target_pose), _target_pose_sub, &_target_pose);
		_target_pose_initialised = true;
	}

	if (_target_pose_initialised) {
		if (_target_pose.rel_vel_valid){
			time_since_last_sighting = (hrt_absolute_time() - _target_pose.timestamp);
			time_since_last_sighting /= SEC2USEC;
		}
	} else
		time_since_last_sighting = 2* _param_target_really_lost_timeout.get();

	// stop if we are landed
	if (_navigator->get_land_detected()->landed) {
		_state = PrecLandState::Done;
	}

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	switch (_state) {
	case PrecLandState::InitSearch: {
		mavlink_log_info(&mavlink_log_pub,"Climbing to search altitude.");
		init_search_triplet();
		_state = PrecLandState::WaitForTarget;
		// fallthrough
	} case PrecLandState::WaitForTarget: {
		// check if we can see the target, and have a valid estimates
		if ( time_since_last_sighting < 1.f ) {
			_state = PrecLandState::InitApproach;
			// fallthrough
		} else
			break;
	} case PrecLandState::InitApproach: {
		mavlink_log_info(&mavlink_log_pub,"Target found. Approaching.");
		_state = PrecLandState::RunApproach;
		// fallthrough
	} case PrecLandState::RunApproach: {
		// check if target visible
		if (time_since_last_sighting < _param_target_lost_timeout.get()) {
			update_approach();
			break;
		} else
			_state = PrecLandState::InitLost;

		break;
	} case PrecLandState::InitLost: {
		mavlink_log_info(&mavlink_log_pub,"Target lost.");

		_state = PrecLandState::RunLost;
		// fallthrough
	} case PrecLandState::RunLost: {
		if ( time_since_last_sighting < 1.f) {
			_state = PrecLandState::InitApproach;
		} else {
			//immidiately increase the area we are looking at
			if (-vehicle_local_position->z < _param_search_alt.get() - 2)
				land_speed_smthr.addSample(-2);
			else
				land_speed_smthr.addSample(0);


			if (no_v_diff_cnt < _param_v_diff_cnt_tresh.get()) {
				//assume that we've lost the target because we weren't moving fast enough
				pos_sp_triplet->current.vx = vx_smthr.get_latest()*2;
				pos_sp_triplet->current.vy = vy_smthr.get_latest()*2;
			}

			//pos_sp_triplet->current.alt = _navigator->get_global_position()->alt - land_speed_smthr.get_latest() ;
			pos_sp_triplet->current.vz = land_speed_smthr.get_latest();
			pos_sp_triplet->current.alt_valid = false;

			pos_sp_triplet->current.velocity_valid = true;
			pos_sp_triplet->current.position_valid = false;
			pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
			pos_sp_triplet->next.valid = false;

			_navigator->set_position_setpoint_triplet_updated();
		}

		if (time_since_last_sighting > _param_target_really_lost_timeout.get())
			_state = PrecLandState::InitSearch;

		break;
	} case PrecLandState::Done: {
		if (!(debug_msg_div % 12))
			mavlink_log_info(&mavlink_log_pub,"Landing done!");
		break;
	} default:
		break;
	}
}

void PrecLand::init_search_triplet() {
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.alt = vehicle_local_position->ref_alt + _param_search_alt.get();
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
	d_angle_x_smthr.init();
	d_angle_y_smthr.init();
	vx_smthr.init(0);
	vy_smthr.init(0);
	land_speed_smthr.init(0.f);
	angle_x_i_err = 0;
	angle_y_i_err = 0;
	no_v_diff_cnt =0;
	_target_pose_initialised = false;
	_target_pose_updated = false;
	_navigator->set_position_setpoint_triplet_updated();
}

void PrecLand::update_land_speed() {

//	std::cout << "t " << time_since_last_sighting;
	if(in_acceptance_range() && time_since_last_sighting < 1.f ){
		float a = sqrtf(powf(_target_pose.angle_x,2)+powf(_target_pose.angle_y,2));

//		std::cout << " a: " << a;

		float a_max = 0.4f; //this is camera (fov) dependent
		//is in the range of approx [0 - a_max], when it is 0 we want to descend as fast a possible.
		//When it's a_max stop descending.

		float max_land_speed = _param_pld_v_lnd.get();
		float h = -_navigator->get_local_position()->z;
		if (h>15)
			max_land_speed *= 2.f;
		if (h<5)
			max_land_speed *= 0.5f;

		float land_speed;
		if (a<0.05f)
			land_speed = max_land_speed;
		else if (a>0.35f)
			land_speed = 0.01f;
		else
			land_speed = max_land_speed * (1.f/a_max) *(a_max-a);

		if (_param_only_flw.get())
			land_speed = 0;
		if (land_speed<0)
			land_speed = 0;

		land_speed_smthr.addSample(land_speed);
		if (!(debug_msg_div % 12))
			mavlink_log_info(&mavlink_log_pub, "Land speed %.2f x %.2f y %.2f H %.2f", static_cast<double>(land_speed), static_cast<double>(_target_pose.angle_x), static_cast<double>(_target_pose.angle_y), static_cast<double>(h));
	} else {

		if (!(debug_msg_div % 12) ){
			bool pos_control_enabled = no_v_diff_cnt <  _param_v_diff_cnt_tresh.get()+2;
			if (pos_control_enabled){
				mavlink_log_info(&mavlink_log_pub, "Catching up %d %d %d x %.2f y %.2f",_target_pose_initialised , _target_pose.rel_vel_valid , in_acceptance_range(), static_cast<double>(_target_pose.angle_x), static_cast<double>(_target_pose.angle_y));
			} else {
				mavlink_log_info(&mavlink_log_pub, "Positioning %d %d %d x %.2f y %.2f",_target_pose_initialised , _target_pose.rel_vel_valid , in_acceptance_range(), static_cast<double>(_target_pose.angle_x), static_cast<double>(_target_pose.angle_y));
			}
		}

		land_speed_smthr.addSample(0.f);
	}
//	std::cout << std::endl;
}

void PrecLand::update_approach() {

	static uint64_t t_prev = 0;
	if (t_prev == _target_pose.timestamp)
		return;

	update_land_speed();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	double lat, lon;
	map_projection_reproject(&_map_ref, _target_pose.x_abs,_target_pose.y_abs, &lat, &lon);

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;
	pos_sp_triplet->current.vx = vx_smthr.addSample(_target_pose.vx_abs);
	pos_sp_triplet->current.vy = vy_smthr.addSample(_target_pose.vy_abs);

	float ss_p_gain = _param_pld_xy_g_p.get(); //pos p gain
	float ss_i_gain = _param_pld_xy_g_i.get()/100.f; // pos i gai
	float ss_d_gain = _param_pld_xy_g_d.get(); //pos p gain
	float i_x_bound = _param_pld_x_bi.get(); //bound pos y control m/s
	float i_y_bound = _param_pld_y_bi.get(); //bound pos y control m/s

	//only activate pos control when drone is up to speed
	float dv = sqrtf(powf(_target_pose.vx_rel,2) + powf(_target_pose.vy_rel,2));

	static float angle_x_prev = _target_pose.angle_x;
	static float angle_y_prev = _target_pose.angle_y;

	float dt = (_target_pose.timestamp - t_prev ) / 1e6f;

	float d_angle_x = _target_pose.angle_x - angle_x_prev;
	float d_angle_y = _target_pose.angle_y - angle_y_prev;

	float d_angle_x_smoothed = 0;
	float d_angle_y_smoothed = 0;
	if (t_prev>0){
		d_angle_x_smoothed = d_angle_x_smthr.addSample(d_angle_x/dt);
		d_angle_y_smoothed = d_angle_y_smthr.addSample(d_angle_y/dt);
	}
	angle_x_prev = _target_pose.angle_x;
	angle_y_prev = _target_pose.angle_y;
	t_prev = _target_pose.timestamp;

	int tresh = _param_v_diff_cnt_tresh.get();

	if (dv<1 && no_v_diff_cnt < tresh+2 && _target_pose.abs_pos_valid && _target_pose_updated)
		no_v_diff_cnt++;
	if (no_v_diff_cnt > tresh){
//		std::cout << "pos control, vz:" << land_speed_smthr.get_latest() << std::endl;
		angle_x_i_err+=_target_pose.angle_x;
		angle_y_i_err+=_target_pose.angle_y;

		float ss_vx = ss_p_gain * _target_pose.angle_x + ss_d_gain*d_angle_x_smoothed + angle_x_i_err * ss_i_gain;
		float ss_vy = ss_p_gain * _target_pose.angle_y + ss_d_gain*d_angle_y_smoothed + angle_y_i_err * ss_i_gain;

		//gain scheduling based on height. 100% gains above 50m height, no lower than 20%
		float f = -_navigator->get_local_position()->z; //_target_pose.z_rel;
		if (f< 0)
			f = 0;
		f = powf(f,1.2f);
		f/= 50;
		if (f>1)
			f = 1;
		else if (f<0.5f)
			f = 0.5f;
		ss_vx*=f;
		ss_vy*=f;

		if (ss_vx>i_x_bound) {
			ss_vx = i_x_bound;
			//				angle_x_i_err = i_x_bound / ss_i_gain;
		} else if (ss_vx<-i_x_bound){
			ss_vx = -i_x_bound;
			//				angle_x_i_err = -i_x_bound / ss_i_gain;
		}
		if (ss_vy>i_y_bound){
			ss_vy = i_y_bound;
			//				angle_y_i_err = i_y_bound / ss_i_gain;
		} else if (ss_vy<-i_y_bound) {
			ss_vy = -i_y_bound;
			//				angle_y_i_err = -i_y_bound / ss_i_gain;
		}

		pos_sp_triplet->current.vx +=ss_vx;
		pos_sp_triplet->current.vy +=ss_vy;
	}

//	pos_sp_triplet->current.alt = _navigator->get_global_position()->alt - land_speed_smthr.get_latest() ;
	pos_sp_triplet->current.vz = land_speed_smthr.get_latest();
	pos_sp_triplet->current.alt_valid = false;

	//land into the direction of the marker (adjust horizontal speed):
	float h = -_navigator->get_local_position()->z;
	float vxr = _target_pose.x_rel / h;
	float vyr = _target_pose.y_rel / h;
	if (vxr > 1)
		vxr = 1;
	else if (vxr < -1)
		vxr = -1;
	if (vyr > 1)
		vyr = 1;
	else if (vyr < -1)
		vyr = -1;
	pos_sp_triplet->current.vx += vxr*land_speed_smthr.get_latest();
	pos_sp_triplet->current.vy += vyr*land_speed_smthr.get_latest();

	pos_sp_triplet->current.velocity_valid = true;
	pos_sp_triplet->current.position_valid = false;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->current.yawspeed_valid = false;
	_navigator->set_position_setpoint_triplet_updated();
}

bool PrecLand::in_acceptance_range() {
	if (_target_pose.abs_pos_valid) {
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
	{ // disable smoothing... to be sure:
		_ready = true;
		_runner = sample;
		return sample;
	}

	_rbuf(_rotater) = sample;                     // overwrite oldest sample in the roundtrip buffer
	_rotater = (_rotater + 1) % (_kernelsize);   //update pointer to buffer
	_runner = _runner + sample - _rbuf(_rotater); //add new sample, subtract the new oldest sample

	if (!_ready)
	{ // check if completely filled
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
	{ // disable smoothing... to be sure:
		_ready = true;
		_runner = sample;
		return sample;
	}

	_rbuf(_rotater) = sample;                     // overwrite oldest sample in the roundtrip buffer
	_rotater = (_rotater + 1) % (_kernelsize);   //update pointer to buffer
	_runner = _runner + sample - _rbuf(_rotater); //add new sample, subtract the new oldest sample

	if (!_ready)
	{ // check if completely filled
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
