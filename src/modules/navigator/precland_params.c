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
 * @file precland_params.c
 *
 * Parameters for precision landing.
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

/**
 * Landing Target Timeout
 *
 * Time after which the landing target is considered lost without any new measurements.
 *
 * @unit s
 * @min 0.0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_BTOUT, 10.0f);

/**
 * Horizontal acceptance radius
 *
 * Start descending if closer above landing target than this.
 *
 * @min 0.0
 * @max 0.64
 * @decimal 2
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_HACC_RAD, 0.15f);

/**
 * Final approach altitude
 *
 * Allow final approach (without horizontal positioning) if losing landing target closer than this to the ground.
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_FAPPR_ALT, 0.1f);

/**
 * Search altitude
 *
 * Altitude above home to which to climb when searching for the landing target.
 *
 * @unit m
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_SRCH_ALT, 10.0f);

/**
 * Only follow
 *
 * @boolean
 * @group System
 */
PARAM_DEFINE_INT32(PLD_ONLY_FLW, 0);

/**
 * Time out during horizontal approach. 
 *
 * Max time that the target is lost seen and predicted based on the v esitmate.
 *
 * @min 1000
 * @max 30000000
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_FLW_TOUT, 10000000);

/**
 * Smoothing filter width velocity predition target
 *
 * For the predition of the movement of the target, a smoothing filter is used.
 *
 * @min 1
 * @max 5000
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_SMT_WDT, 100);

/**
 * Search timeout
 *
 * Time allowed to search for the landing target before falling back to normal landing.
 *
 * @unit s
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_SRCH_TOUT, 10.0f);

/**
 * Maximum number of search attempts
 *
 * Maximum number of times to seach for the landing target if it is lost during the precision landing.
 *
 * @min 0
 * @max 100
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_MAX_SRCH, 3);

/**
 * v_diff_cnt_tresh
 *
 * Maximum number of times to seach for the landing target if it is lost during the precision landing.
 *
 * @min 5
 * @max 1000
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_VD_CNT, 100);

/**
 * Position P gain
 *
 * @min 0.0
 * @max 50
 * @decimal 1
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_P_XY_G, 3.0f);

/**
 * Position I gain
 *
 * @min 0.0
 * @max 50
 * @decimal 1
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_I_XY_G, 0.00f);

/**
 * Position D gain
 * (NOT USED AT THE MOMENT)
 * @min 0.0
 * @max 50
 * @decimal 1
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_D_XY_G, 0.8f);

/**
 * Bound speed for position control in x direction
 *
 * @unit m/s
 * @min 0.0
 * @max 5
 * @decimal 1
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_I_X_B, 1.5f);

/**
 * Bound speed for position control in y direction
 *
 * @unit m/s
 * @min 0.0
 * @max 5
 * @decimal 1
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_I_Y_B, 2.5f);

/**
 * Land speed. Above 15m this number is doubled. Below 4 meters this number is halved.
 *
 * @unit m/s
 * @min 0.0
 * @max 5
 * @decimal 1
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_V_LND, 1.5f);
			       