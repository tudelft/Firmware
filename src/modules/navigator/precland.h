/***************************************************************************
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
 * @file precland.h
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#pragma once

#include <matrix/math.hpp>
#include <lib/ecl/geo/geo.h>
#include <px4_module_params.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/distance_sensor.h>

#include "navigator_mode.h"
#include "mission_block.h"

#include <vector>




/*
 * This class performs moving average filtering
 *
 */
class Smoother
{

  private:
	matrix::Vector<float,1000> _rbuf; // rotary buffer
	int _kernelsize;          // filter kernel width
	int _rotater;             //pointer to current sample in rotary buffer
	float _runner;            // current filter output value
	bool _ready = false;

  public:
	void init(int width);
	void init(int width, float value);
	float addSample(float sample);
	float get_latest();
	void reset(void);
	void reset_to(float v);
	bool ready()
	{
		return _ready;
	}
	int kernselsize() { return _kernelsize;}
};



enum class PrecLandState {
	Search, // Search for landing target
	HorizontalApproach, // Positioning over landing target while maintaining altitude
	Done // Done landing
};

enum class PrecLandMode {
	Opportunistic = 1, // only do precision landing if landing target visible at the beginning
	Required = 2 // try to find landing target if not visible at the beginning
};

class PrecLand : public MissionBlock, public ModuleParams
{
public:
	PrecLand(Navigator *navigator);
	~PrecLand() override = default;

	void on_activation() override;
	void on_active() override;

	void set_mode(PrecLandMode mode) {
		_mode = PrecLandMode::Required;
	};

	PrecLandMode get_mode() { return _mode; };

private:
	// run the control loop for each state
	void run_state_horizontal_approach();
	void run_state_search();

	void update_postriplet(float px, float py);
	bool in_acceptance_range();

	// attempt to switch to a different state. Returns true if state change was successful, false otherwise
	bool switch_to_state_horizontal_approach();
	bool switch_to_state_search();
	bool switch_to_state_done();

	// check if a given state could be changed into. Return true if possible to transition to state, false otherwise
	bool check_state_conditions(PrecLandState state);
	void predict_target();

	landing_target_pose_s _target_pose{}; /**< precision landing target position */

	distance_sensor_s ds_report = {};

	int _target_pose_sub{-1};
	bool _target_pose_valid{false}; /**< whether we have received a landing target position message */
	bool _target_pose_updated{false}; /**< wether the landing target position message is updated */

	int _stereo_height_sub{-1};
	bool _stereo_height_updated{false}; /**< wether the height message is updated */

	struct map_projection_reference_s _map_ref {}; /**< reference for local/global projections */

	uint64_t _state_start_time{0}; /**< time when we entered current state */
	uint64_t _last_slewrate_time{0}; /**< time when we last limited setpoint changes */
	uint64_t _target_acquired_time{0}; /**< time when we first saw the landing target during search */
	uint64_t _point_reached_time{0}; /**< time when we reached a setpoint */


	orb_advert_t mavlink_log_pub = nullptr;

	float angle_x_i_err = 0;
	float angle_y_i_err = 0;
	int no_v_diff_cnt;

	int count_div = 0; // tmp

	int _search_cnt{0}; /**< counter of how many times we had to search for the landing target */
	float _approach_alt{0.0f}; /**< altitude at which to stay during horizontal approach */

	matrix::Vector2f _sp_pev;
	matrix::Vector2f _sp_pev_prev;

	Smoother land_speed_smthr, vx_smthr,vy_smthr,d_angle_x_smthr,d_angle_y_smthr,boat_wave_z_speed_smthr;
	float last_good_target_pose_x;
	float last_good_target_pose_y;
	uint64_t last_good_target_pose_time;

	PrecLandState _state{PrecLandState::Search};

	PrecLandMode _mode{PrecLandMode::Required};

	DEFINE_PARAMETERS(
			(ParamFloat<px4::params::PLD_BTOUT>) _param_timeout,
			(ParamFloat<px4::params::PLD_HACC_RAD>) _param_hacc_rad,
			(ParamFloat<px4::params::PLD_FAPPR_ALT>) _param_final_approach_alt,
			(ParamFloat<px4::params::PLD_SRCH_ALT>) _param_search_alt,
			(ParamInt<px4::params::PLD_ONLY_FLW>) _param_only_flw,
			(ParamInt<px4::params::PLD_FLW_TOUT>) _param_flw_tout,
			(ParamInt<px4::params::PLD_SMT_WDT>) _param_smt_wdt,
			(ParamFloat<px4::params::PLD_P_XY_G>) _param_pld_p_xy_g,
			(ParamFloat<px4::params::PLD_I_XY_G>) _param_pld_i_xy_g,
			(ParamFloat<px4::params::PLD_D_XY_G>) _param_pld_d_xy_g,
			(ParamFloat<px4::params::PLD_I_X_B>) _param_pld_i_x_b,
			(ParamFloat<px4::params::PLD_I_Y_B>) _param_pld_i_y_b,
			(ParamFloat<px4::params::PLD_V_LND>) _param_pld_v_lnd,
			(ParamFloat<px4::params::PLD_SRCH_TOUT>) _param_search_timeout,
			(ParamInt<px4::params::PLD_MAX_SRCH>) _param_max_searches,
			(ParamInt<px4::params::PLD_VD_CNT>) _param_v_diff_cnt_tresh,
			(ParamFloat<px4::params::MPC_ACC_HOR>) _param_acceleration_hor,
			(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_xy_vel_cruise
			)

};
