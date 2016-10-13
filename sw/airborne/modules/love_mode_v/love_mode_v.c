/*
 * Copyright (C) wudong
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/love_mode_v/love_mode_v.c"
 * @author wudong
 * @e-mail 862281335@qq.com
 * test the us100 sonar in ppz
 */

#include "state.h"
#include "subsystems/ins/vf_float.h"
#include "modules/sonar_for_ppz/sonar_for_ppz.h"
#include "modules/love_mode_v/love_mode_v.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization.h"


#ifndef MAX_PPRZ
#define MAX_PPRZ 9600
#endif

#ifndef GUIDANCE_V_MIN_ERR_Z
#define GUIDANCE_V_MIN_ERR_Z POS_BFP_OF_REAL(-10.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_Z
#define GUIDANCE_V_MAX_ERR_Z POS_BFP_OF_REAL(10.)
#endif

#ifndef GUIDANCE_V_MIN_ERR_ZD
#define GUIDANCE_V_MIN_ERR_ZD SPEED_BFP_OF_REAL(-10.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_ZD
#define GUIDANCE_V_MAX_ERR_ZD SPEED_BFP_OF_REAL(10.)
#endif

#ifndef GUIDANCE_V_MAX_SUM_ERR
#define GUIDANCE_V_MAX_SUM_ERR 2000000
#endif

#define GuidanceVSetRef(_pos, _speed, _accel) { \
    gv_set_ref(_pos, _speed, _accel);        \
    guidance_v_z_ref = _pos;             \
    guidance_v_zd_ref = _speed;          \
    guidance_v_zdd_ref = _accel;             \
  }


#define FF_CMD_FRAC 18
static void run_hover_loop(bool in_flight);


void control_v_enter(void) 
{

	guidance_v_z_sp = 256*sonar_height;//stateGetPositionNed_i()->z; // set current altitude as setpoint
        guidance_v_z_sum_err = 0;
	GuidanceVSetRef(256*sonar_height, 0, 0);
        //GuidanceVSetRef(stateGetPositionNed_i()->z, 0, 0);
}


/*      why we don't need to read rc in love_mode vertical?
 *	guidance_v_read_rc() in the guidance_v.c line 203.	
 *	the result just calculate the rc_climb scale.
 *	we don't need now.
 *	and guidance_v_read_rc() and guidance_h_read_rc()
 *	always be called at autopilot.c line 829.
 */

void control_v_run(bool in_flight)
{

 	guidance_v_zd_sp = 0;
	gv_update_ref_from_z_sp(guidance_v_z_sp);
	run_hover_loop(in_flight);
	stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
}


/** controls for Vertical loops **/
static void run_hover_loop(bool in_flight)
{

  	int64_t tmp  = gv_z_ref >> (GV_Z_REF_FRAC - INT32_POS_FRAC);
  	guidance_v_z_ref = (int32_t)tmp;
  	guidance_v_zd_ref = gv_zd_ref << (INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
  	guidance_v_zdd_ref = gv_zdd_ref << (INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
 
  	//int32_t err_z  = guidance_v_z_ref - stateGetPositionNed_i()->z;
	int32_t err_z  = guidance_v_z_ref - sonar_height*256;//use the sonar
  	Bound(err_z, GUIDANCE_V_MIN_ERR_Z, GUIDANCE_V_MAX_ERR_Z);
  	int32_t err_zd = guidance_v_zd_ref - stateGetSpeedNed_i()->z;
  	Bound(err_zd, GUIDANCE_V_MIN_ERR_ZD, GUIDANCE_V_MAX_ERR_ZD);

  	if (in_flight) {
    	guidance_v_z_sum_err += err_z;
    	Bound(guidance_v_z_sum_err, -GUIDANCE_V_MAX_SUM_ERR, GUIDANCE_V_MAX_SUM_ERR);
  	} else {
    	guidance_v_z_sum_err = 0;
  	}

  	int32_t inv_m;
  	if (guidance_v_adapt_throttle_enabled) {
    	inv_m =  gv_adapt_X >> (GV_ADAPT_X_FRAC - FF_CMD_FRAC);
  	} else {
   
    	inv_m = BFP_OF_REAL(9.81 / (guidance_v_nominal_throttle * MAX_PPRZ), FF_CMD_FRAC);
  	}

  	const int32_t g_m_zdd = (int32_t)BFP_OF_REAL(9.81, FF_CMD_FRAC) -
                         	 (guidance_v_zdd_ref << (FF_CMD_FRAC - INT32_ACCEL_FRAC));

  	guidance_v_ff_cmd = g_m_zdd / inv_m;

  	guidance_v_ff_cmd = (guidance_v_ff_cmd << INT32_TRIG_FRAC) / guidance_v_thrust_coeff;

  	Bound(guidance_v_ff_cmd, 0, 8640);

  	guidance_v_fb_cmd = ((-guidance_v_kp * err_z)  >> 7) +
                      	((-guidance_v_kd * err_zd) >> 16) +
                      	((-guidance_v_ki * guidance_v_z_sum_err) >> 16);

  	guidance_v_delta_t = guidance_v_ff_cmd + guidance_v_fb_cmd;

  	Bound(guidance_v_delta_t, 0, MAX_PPRZ);
}
