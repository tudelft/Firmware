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
 * Landing Target Lost Timeout
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
PARAM_DEFINE_FLOAT(PLD_TLST_TOUT, 3.0f);

/**
 * Landing Target Really Lost Timeout
 *
 * Time after which the landing target is considered really lost after which the search is reset
 *
 * @unit s
 * @min 0.0
 * @max 120
 * @decimal 1
 * @increment 0.5
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_RLST_TOUT, 20.0f);

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
 * Horizontal acceptance radius
 *
 * Start descending if target in the center of image.
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
 * Only follow, don't descend
 *
 * @boolean
 * @group System
 */
PARAM_DEFINE_INT32(PLD_ONLY_FLW, 0);

/**
 * v_diff_cnt_tresh
 *
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
PARAM_DEFINE_FLOAT(PLD_XY_G_P, 3.0f);

/**
 * Position I gain
 *
 * @min 0.0
 * @max 50
 * @decimal 1
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_XY_G_I, 0.00f);

/**
 * Position D gain
 * (NOT USED AT THE MOMENT)
 * @min 0.0
 * @max 50
 * @decimal 1
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_XY_G_D, 0.8f);

/**
 * Position P gain
 *
 * @min 1.0
 * @max 500
 * @decimal 1
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_XY_SHP, 60.0f);


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

/**
 * v up speed after lost. If the marker was found, but lost after time out. The drone goes up with max this speed.
 *
 * @unit m/s
 * @min -5.0
 * @max -0.0
 * @decimal 1
 * @increment 0.05
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_V_U_LST, -0.5f);

PARAM_DEFINE_INT32(PLD_DST_TYPE, 0);
