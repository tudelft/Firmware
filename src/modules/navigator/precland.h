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
class Smoother_100
{
#define SMOOTHER_100_WIDTH 100
  private:
	matrix::Vector<float,SMOOTHER_100_WIDTH+1> _rbuf; // rotary buffer
	int _kernelsize;          // filter kernel width
	int _rotater;             //pointer to current sample in rotary buffer
	float _runner;            // current filter output value
	bool _ready = false;

  public:
	Smoother_100() {
		_kernelsize = SMOOTHER_100_WIDTH;
	}
	void init();
	void init(float value);
	float addSample(float sample);
	float get_latest();
	void reset(void);
	void reset_to(float v);
	bool ready() {
		return _ready;
	}
	int kernselsize() { return _kernelsize;}
};

/*
 * This class performs moving average filtering
 *
 */
class Smoother_10
{
#define SMOOTHER_10_WIDTH 10
  private:
	matrix::Vector<float,SMOOTHER_10_WIDTH+1> _rbuf; // rotary buffer
	int _kernelsize;          // filter kernel width
	int _rotater;             //pointer to current sample in rotary buffer
	float _runner;            // current filter output value
	bool _ready = false;

  public:
	Smoother_10() {
		_kernelsize = SMOOTHER_10_WIDTH;
	}
	void init();
	void init(float value);
	float addSample(float sample);
	float get_latest();
	void reset(void);
	void reset_to(float v);
	bool ready() {
		return _ready;
	}
	int kernselsize() { return _kernelsize;}
};


enum class PrecLandState {
	InitSearch,
	WaitForTarget,
	InitApproach,
	RunApproach,
	InitLost,
	RunLost,
	Done
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
	void init_search_triplet();
	void update_approach_land_speed();
	void update_approach();
	bool in_acceptance_range();

	landing_target_pose_s _target_pose{}; /**< precision landing target position */

	int _target_pose_sub{-1};
	bool _target_pose_initialised{false}; /**< whether we have received a landing target position message */
	bool _target_pose_updated{false}; /**< wether the landing target position message is updated */
	struct map_projection_reference_s _map_ref {}; /**< reference for local/global projections */

	Smoother_10 d_angle_x_smthr,d_angle_y_smthr,land_speed_smthr;

	float angle_x_i_err = 0; //TODO: i controller may not be needed. Remove?
	float angle_y_i_err = 0;
	int no_v_diff_cnt;
	float time_since_last_sighting = 999;
	Smoother_100 vx_smthr,vy_smthr;

	int debug_msg_div = 0; // divider counter to limit debug messages
	orb_advert_t mavlink_log_pub = nullptr;

	PrecLandState _state{PrecLandState::InitSearch};

	PrecLandMode _mode{PrecLandMode::Required};

	DEFINE_PARAMETERS(
			(ParamFloat<px4::params::PLD_TLST_TOUT>) _param_target_lost_timeout,
			(ParamFloat<px4::params::PLD_RLST_TOUT>) _param_target_really_lost_timeout,
			(ParamFloat<px4::params::PLD_SRCH_ALT>) _param_search_alt,
			(ParamFloat<px4::params::PLD_HACC_RAD>) _param_hacc_rad,
			(ParamFloat<px4::params::PLD_FAPPR_ALT>) _param_final_approach_alt,
			(ParamInt<px4::params::PLD_ONLY_FLW>) _param_only_flw,
			(ParamFloat<px4::params::PLD_XY_G_P>) _param_pld_xy_g_p,
			(ParamFloat<px4::params::PLD_XY_G_I>) _param_pld_xy_g_i,
			(ParamFloat<px4::params::PLD_XY_G_D>) _param_pld_xy_g_d,
			(ParamFloat<px4::params::PLD_X_BI>) _param_pld_x_bi,
			(ParamFloat<px4::params::PLD_Y_BI>) _param_pld_y_bi,
			(ParamFloat<px4::params::PLD_V_LND>) _param_pld_v_lnd,
			(ParamInt<px4::params::PLD_VD_CNT>) _param_v_diff_cnt_tresh
			)

};
