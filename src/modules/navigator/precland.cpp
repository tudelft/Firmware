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

#define STATE_TIMEOUT 100000000 // [us] Maximum time to spend in any state

PrecLand::PrecLand(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
PrecLand::on_activation()
{
	d_angle_x_smthr.init(10);
	d_angle_y_smthr.init(10);
	vx_smthr.init(_param_smt_wdt.get());
	vy_smthr.init(_param_smt_wdt.get());
	land_speed_smthr.init(_param_smt_wdt.get(),0.f);
	last_good_target_pose_time = 0;
	angle_x_i_err = 0;
	angle_y_i_err = 0;
	no_v_diff_cnt =0;
	_target_pose_valid = false;
	_target_pose_updated = false;

	// We need to subscribe here and not in the constructor because constructor is called before the navigator task is spawned
	if (_target_pose_sub < 0) {
		_target_pose_sub = orb_subscribe(ORB_ID(landing_target_pose));
	}

	_search_cnt = 0;
	_last_slewrate_time = 0;

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (!map_projection_initialized(&_map_ref)) {
		map_projection_init(&_map_ref, vehicle_local_position->ref_lat, vehicle_local_position->ref_lon);
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->next.valid = false;

	// Check that the current position setpoint is valid, otherwise land at current position
	if (!pos_sp_triplet->current.valid) {
		mavlink_log_info(&mavlink_log_pub,"Resetting landing position to current position");
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.valid = true;
	}

	_sp_pev = matrix::Vector2f(0, 0);
	_sp_pev_prev = matrix::Vector2f(0, 0);
	_last_slewrate_time = 0;

	switch_to_state_search();

}

void
PrecLand::on_active()
{
	// get new target measurement
	orb_check(_target_pose_sub, &_target_pose_updated);

	if (_target_pose_updated) {
		orb_copy(ORB_ID(landing_target_pose), _target_pose_sub, &_target_pose);
		_target_pose_valid = true;
	}

	if ((hrt_elapsed_time(&_target_pose.timestamp) / 1e6f) > _param_timeout.get()) {
		_target_pose_valid = false;
	}

	// stop if we are landed
	if (_navigator->get_land_detected()->landed) {
		switch_to_state_done();
	}

	//predict (moving) target location, if target is in sight
	predict_target();

	switch (_state) {
	case PrecLandState::Search:
		run_state_search();
		break;

	case PrecLandState::HorizontalApproach:
		run_state_horizontal_approach();
		break;

	case PrecLandState::Done:
		// nothing to do
		break;

	default:
		// unknown state
		break;
	}

}

void PrecLand::predict_target() {
	count_div++;
	float time_since_last_sighting = (hrt_absolute_time() - last_good_target_pose_time);
	time_since_last_sighting /=SEC2USEC;
	if(_target_pose_valid && _target_pose.rel_vel_valid && in_acceptance_range() ){
		float a = sqrtf(powf(_target_pose.angle_x,2)+powf(_target_pose.angle_y,2));

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
		if (!(count_div % 12))
			mavlink_log_info(&mavlink_log_pub, "Land speed %.2f x %.2f y %.2f H %.2f t %.2f", static_cast<double>(land_speed), static_cast<double>(_target_pose.angle_x), static_cast<double>(_target_pose.angle_y), static_cast<double>(h),static_cast<double>(time_since_last_sighting));
	} else {

		if (!(count_div % 12) && _target_pose.rel_pos_valid)
			mavlink_log_info(&mavlink_log_pub, "NOT IN LANDING ZONE %d %d %d x %.2f y %.2f t %.2f",_target_pose_valid , _target_pose.rel_vel_valid , in_acceptance_range(), static_cast<double>(_target_pose.angle_x), static_cast<double>(_target_pose.angle_y), static_cast<double>(time_since_last_sighting));


		land_speed_smthr.addSample(0.f);
	}
	//	std::cout << "angle: " << _target_pose.angle_x << ", " << _target_pose.angle_y
	//		  << " land speed: " << land_speed_smthr.get_latest() << std::endl;

	if(_target_pose_valid && _target_pose.abs_pos_valid){
		last_good_target_pose_time = hrt_absolute_time();
	}
}

void PrecLand::update_postriplet(float px, float py){
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	double lat, lon;
	map_projection_reproject(&_map_ref, px, py, &lat, &lon);

	uint64_t now = hrt_absolute_time();
	float time_since_last_sighting = (now - last_good_target_pose_time);
	time_since_last_sighting /= SEC2USEC;	

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;
	pos_sp_triplet->current.vx = vx_smthr.addSample(_target_pose.vx_abs);
	pos_sp_triplet->current.vy = vy_smthr.addSample(_target_pose.vy_abs);

	float ss_p_gain = _param_pld_p_xy_g.get(); //pos p gain
	float ss_i_gain = _param_pld_i_xy_g.get()/100.f; // pos i gai
	float ss_d_gain = _param_pld_d_xy_g.get(); //pos p gain
	float i_x_bound = _param_pld_i_x_b.get(); //bound pos y control m/s
	float i_y_bound = _param_pld_i_y_b.get(); //bound pos y control m/s

	//only activate pos control when drone is up to speed
	float dv = sqrtf(powf(_target_pose.vx_rel,2) + powf(_target_pose.vx_rel,2));

	static float angle_x_prev = _target_pose.angle_x;
	static float angle_y_prev = _target_pose.angle_y;
	static uint64_t t_prev = 0;

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

	if (dv<1 && no_v_diff_cnt < tresh+2 && _target_pose.abs_pos_valid)
		no_v_diff_cnt++;
	if (no_v_diff_cnt > tresh){
		angle_x_i_err+=_target_pose.angle_x;
		angle_y_i_err+=_target_pose.angle_y;

		float ss_vx = ss_p_gain * _target_pose.angle_x + ss_d_gain*d_angle_x_smoothed + angle_x_i_err * ss_i_gain;
		float ss_vy = ss_p_gain * _target_pose.angle_y + ss_d_gain*d_angle_y_smoothed + angle_y_i_err * ss_i_gain;

		//gain scheduling based on height. 100% gains above 50m height, no lower than 20%
		float f = -_navigator->get_local_position()->z; //_target_pose.z_rel;
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
		//			std::cout << "vel smoothed: "  << pos_sp_triplet->current.vx << ", " << pos_sp_triplet->current.vy
		//				  << " angle g: "  <<ss_vx << ", " << ss_vy
		//				  << " last sighting: " << time_since_last_sighting  << std::endl;
	}

	pos_sp_triplet->current.velocity_valid = true;
	pos_sp_triplet->current.position_valid = false;

	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
		//		std::cout << "Follow pos: " << px << ", " << py << " vel smoothed: "  << pos_sp_triplet->current.vx << ", " << pos_sp_triplet->current.vy
		//			  << " vel raw: "  << _target_pose.vx_abs << ", " << _target_pose.vy_abs
		//			  << " last sighting: " << time_since_last_sighting  << std::endl;



	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->current.yawspeed_valid = false;
//		pos_sp_triplet->current.yawspeed = yaw_rate;

	

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


	_navigator->set_position_setpoint_triplet_updated();
}

void
PrecLand::run_state_horizontal_approach()
{

	// check if target visible, if not go to start
	if (!check_state_conditions(PrecLandState::HorizontalApproach)) {
		mavlink_log_info(&mavlink_log_pub,"Lost landing target while landing (horizontal approach).");

		switch_to_state_search();

		return;
	}

	update_postriplet(_target_pose.x_abs,_target_pose.y_abs);
}

void
PrecLand::run_state_search()
{
	// check if we can see the target
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		if (!_target_acquired_time) {
			// target just became visible. Stop climbing, but give it some margin so we don't stop too apruptly
			_target_acquired_time = hrt_absolute_time();
			position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
			float new_alt = _navigator->get_global_position()->alt + 1.0f;
			pos_sp_triplet->current.alt = new_alt < pos_sp_triplet->current.alt ? new_alt : pos_sp_triplet->current.alt;
			_navigator->set_position_setpoint_triplet_updated();
		}
	}

	// stay at that height for a second to allow the vehicle to settle
	if (_target_acquired_time && (hrt_absolute_time() - _target_acquired_time) > 1000000) {
		// try to switch to horizontal approach
		if (switch_to_state_horizontal_approach()) {
			return;
		}
	}

	// check if search timed out and go to fallback
	if (hrt_absolute_time() - _state_start_time > _param_search_timeout.get()*SEC2USEC) {
		mavlink_log_info(&mavlink_log_pub,"Search timed out");

	}
}

bool
PrecLand::switch_to_state_horizontal_approach()
{
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		_approach_alt = _navigator->get_global_position()->alt;

		_point_reached_time = 0;

		_state = PrecLandState::HorizontalApproach;
		_state_start_time = hrt_absolute_time();

		return true;
	}

	return false;
}


bool
PrecLand::switch_to_state_search()
{
	mavlink_log_info(&mavlink_log_pub,"Climbing to search altitude.");
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.alt = vehicle_local_position->ref_alt + _param_search_alt.get();
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_navigator->set_position_setpoint_triplet_updated();

	_target_acquired_time = 0;

	_state = PrecLandState::Search;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_done()
{
	_state = PrecLandState::Done;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool PrecLand::in_acceptance_range() {
	if (_target_pose.abs_pos_valid) {
		float a = sqrtf(powf(_target_pose.angle_x,2)+powf(_target_pose.angle_y,2));
		return (a<_param_hacc_rad.get() && no_v_diff_cnt>=_param_v_diff_cnt_tresh.get() );
	} else {
		return false;
	}
}

bool PrecLand::check_state_conditions(PrecLandState state)
{
	uint64_t now = hrt_absolute_time();
	float time_since_last_sighting = (now - last_good_target_pose_time);
	time_since_last_sighting /= SEC2USEC;

	switch (state) {
	case PrecLandState::Search:
		return true;
	case PrecLandState::HorizontalApproach:
		// if we're already in this state, only want to make it invalid if we reached the target but can't see it anymore
		if (_state == PrecLandState::HorizontalApproach)
			return time_since_last_sighting < _param_timeout.get();
		else
			// If we're trying to switch to this state, the target needs to be visible
			return _target_pose_updated && _target_pose_valid && _target_pose.abs_pos_valid && _target_pose.rel_vel_valid;
	default:
		return false;
	}
}

void Smoother::init(int width, float value)
{
	if (width > 1000)
		width = 1000;
	_kernelsize = width;
//	_rbuf.resize(_kernelsize + 1);
	_rotater = 0;
	_runner = value*_kernelsize;

	for (int i = 0; i <= _kernelsize; i++)
	{
		_rbuf(i) = value;
	}
	_ready = true;
}

void Smoother::init(int width)
{
	init(width,0);
	_ready = false;
}

void Smoother::reset()
{
	_rotater = 0;
	for (int i = 0; i <= _kernelsize; i++)
	{
		_rbuf(i) = 0;
	}
	_runner = 0;
	_ready = false;
}
void Smoother::reset_to(float v)
{
	_rotater = 0;
	for (int i = 0; i <= _kernelsize; i++)
	{
		_rbuf(i) = v;
	}
	_runner = v*_kernelsize;
	_ready = true;
}

float Smoother::addSample(float sample)
{
	//performs online smoothing filter

	if (isnan(sample)) // fixes nan, which forever destroy the output
		sample = 0;
	if (_kernelsize == 1)
	{ // disable smoothing... to be sure:
		_ready = true;
		_runner = sample;
		return sample;
	}

	_rbuf(_rotater) = sample;                     // overwrite oldest sample in the roundtrip buffer
	_rotater = (_rotater + 1) % (_kernelsize + 1);   //update pointer to buffer
	_runner = _runner + sample - _rbuf(_rotater); //add new sample, subtract the new oldest sample

	if (!_ready)
	{ // check if completely filled
		if (_rotater == 0)
			_ready = true;
		else
			return _runner / _rotater; // if not filled completely, return average over the amount of added data (the rest of the filter is initialised to zero)
	}

	return _runner / _kernelsize;
}

float Smoother::get_latest()
{
	return _runner / _kernelsize;
}


