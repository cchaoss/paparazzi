/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/ctrl_module_demo.h
 * @brief example empty controller
 *
 */

#include "modules/module_choice/module_choice.h"
#include "led.h"
#include "state.h"
#include "modules/ctrl/ctrl_module_demo.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "generated/airframe.h"
#include "subsystems/ins/vf_float.h"
#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

/********************************************************/
#ifndef GUIDANCE_V_CLIMB_RC_DEADBAND
#define GUIDANCE_V_CLIMB_RC_DEADBAND MAX_PPRZ/10
#endif

#ifndef GUIDANCE_V_MAX_RC_CLIMB_SPEED
#define GUIDANCE_V_MAX_RC_CLIMB_SPEED GUIDANCE_V_REF_MIN_ZD
#endif

#ifndef GUIDANCE_V_MAX_RC_DESCENT_SPEED
#define GUIDANCE_V_MAX_RC_DESCENT_SPEED GUIDANCE_V_REF_MAX_ZD
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
/*******************************************************/
#ifndef SAFE_DESCENT_SPEED
#define SAFE_DESCENT_SPEED 0.5 //单位：m/s
#endif

#ifndef TAKE_OFF_SPEED
#define TAKE_OFF_SPEED -0.8
#endif

#ifndef TAKE_OFF_ALTITUDE
#define TAKE_OFF_ALTITUDE 4
#endif

/*******************************************************/
struct ctrl_module_demo_struct {
  int rc_x;
  int rc_y;
  int rc_z;
  int rc_t;
} ctrl_module_demo;
float ctrl_module_demo_pr_ff_gain = 0.2f;  // Pitch/Roll
float ctrl_module_demo_pr_d_gain = 0.1f;
float ctrl_module_demo_y_ff_gain = 0.4f;   // Yaw
float ctrl_module_demo_y_d_gain = 0.05f;

void ctrl_module_init(void);
void ctrl_module_run(bool in_flight);
void stabilization_attitude_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);
void stabilization_attitude_enter(void);
void stabilization_attitude_run(bool in_flight);
static void run_hover_loop(bool in_flight);
/*******************************************************/


float height_bis = 0;
int32_t my_guidance_v_rc_delta_t;
int32_t my_guidance_v_rc_zd_sp;
unsigned char 	take_off  = 1,
	      		rc_climb  = 2,
	      		auto_land = 3,
	      		flag = 0;

/*******************************************Horizontal loops*******************************************************/
void guidance_h_module_init(void)
{

}

void guidance_h_module_enter(void)//GUIDANCE_H_MODE_ATTITUDE
{
          stabilization_attitude_enter();
}


void guidance_h_module_read_rc(void)
{
	/***  guidance_h_read_rc  ***/										
	stabilization_attitude_read_rc(autopilot_in_flight, FALSE, FALSE);//GUIDANCE_H_MODE_ATTITUDE
	
	/***  guidance_v_read_rc  ***/
	//guidance_v_read_rc();//at the autopilot.c line 812
  	my_guidance_v_rc_delta_t = (int32_t)radio_control.values[RADIO_THROTTLE];

  	my_guidance_v_rc_zd_sp = (MAX_PPRZ / 2) - (int32_t)radio_control.values[RADIO_THROTTLE];
  	DeadBand(my_guidance_v_rc_zd_sp, GUIDANCE_V_CLIMB_RC_DEADBAND);

  	static const int32_t climb_scale = ABS(SPEED_BFP_OF_REAL(GUIDANCE_V_MAX_RC_CLIMB_SPEED) /
                                         (MAX_PPRZ / 2 - GUIDANCE_V_CLIMB_RC_DEADBAND));
  	static const int32_t descent_scale = ABS(SPEED_BFP_OF_REAL(GUIDANCE_V_MAX_RC_DESCENT_SPEED) /
                                       (MAX_PPRZ / 2 - GUIDANCE_V_CLIMB_RC_DEADBAND));

  	if (my_guidance_v_rc_zd_sp > 0) {
    	my_guidance_v_rc_zd_sp *= descent_scale;
  	} else {
    	my_guidance_v_rc_zd_sp *= climb_scale;
  	}
	

}


void guidance_h_module_run(bool in_flight)
{

	stabilization_attitude_run(in_flight);//GUIDANCE_H_MODE_ATTITUDE
}



/*********************************************Vertical loops***************************************************************/
void guidance_v_module_init(void)
{
	if(vff.z < -0.1 || vff.z > 0.1)
	{
		for(uint8_t i = 0;i<5;i++)	height_bis += vff.z;
		height_bis /= 5;
	}
}

void guidance_v_module_enter(void)
{
	
#if 1
	if(module_number == take_off)
	{
		guidance_v_zd_sp = 0;//enter climb

		guidance_v_z_sp = -256*TAKE_OFF_ALTITUDE;//stateGetPositionNed_i()->z;//enter hover_z_hold
        guidance_v_z_sum_err = 0;
        GuidanceVSetRef(-256*TAKE_OFF_ALTITUDE, 0, 0);
	}

	if (module_number == rc_climb)
	{	
		guidance_v_zd_sp = 0;
	}

	if (module_number == auto_land)//GUIDANCE_V_MODE_CLIMB
	{
		guidance_v_zd_sp = 0;
		guidance_v_z_sp = -256*height_bis;//stateGetPositionNed_i()->z;//enter hover_z_hold
    	guidance_v_z_sum_err = 0;
    	GuidanceVSetRef(-256*height_bis, 0, 0);
	}
#endif	
}

void guidance_v_module_run(UNUSED bool in_flight)
{

#if 1
	if(module_number == take_off)
	{

		if((-vff.z) < TAKE_OFF_ALTITUDE )//climb with TAKE_OFF_SPEED
		{
			guidance_v_zd_sp = SPEED_BFP_OF_REAL(TAKE_OFF_SPEED);
     		//gv_update_ref_from_zd_sp(guidance_v_zd_sp, stateGetPositionNed_i()->z);
			gv_update_ref_from_zd_sp(guidance_v_zd_sp, stateGetPositionNed_i()->z);
      		run_hover_loop(in_flight);
			stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
		}

		if((-vff.z) >= TAKE_OFF_ALTITUDE-1 && (-vff.z) <= TAKE_OFF_ALTITUDE)//climb with 0.5m/s
		{
				guidance_v_zd_sp = SPEED_BFP_OF_REAL(0.5);
      			gv_update_ref_from_z_sp(guidance_v_z_sp);
      			run_hover_loop(in_flight);
				stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
		}

		if((-vff.z) >= TAKE_OFF_ALTITUDE)//hover	
		{
				guidance_v_zd_sp = 0;
      			gv_update_ref_from_z_sp(guidance_v_z_sp);
      			run_hover_loop(in_flight);
				stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
		}
		

		if((-vff.z) > TAKE_OFF_ALTITUDE + 1)//down with 0.5m/s
		{
			guidance_v_zd_sp = SPEED_BFP_OF_REAL(-0.5);
     		gv_update_ref_from_z_sp(guidance_v_z_sp);
	      	run_hover_loop(in_flight);
			stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
		}

		//gpio_setup_output(GPIOC, GPIO12);// LED_red 亮
		//gpio_set(GPIOC, GPIO12);
	}


/*
	if (module_number == rc_climb)
	{
		
		guidance_v_zd_sp = my_guidance_v_rc_zd_sp;//GUIDANCE_V_MODE_RC_CLIMB
     	gv_update_ref_from_zd_sp(guidance_v_zd_sp, stateGetPositionNed_i()->z);
      	run_hover_loop(in_flight);
      	stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;

		//gpio_setup_output(GPIOC, GPIO12);
		//gpio_toggle(GPIOC, GPIO12);
	}
*/


	if (module_number == auto_land)
	{
		guidance_v_zd_sp = SPEED_BFP_OF_REAL(SAFE_DESCENT_SPEED);//climb
     	gv_update_ref_from_zd_sp(guidance_v_zd_sp, stateGetPositionNed_i()->z);
      	run_hover_loop(in_flight);
        stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
		
		if((vff.z-height_bis) < 0.3)
		//if (!autopilot_in_flight) //在接近地面30cm的时候，还会出现上升现象
		{
			autopilot_set_motors_on(FALSE);
		    stabilization_cmd[COMMAND_THRUST] = 0;
		}	
		
		//gpio_setup_output(GPIOC, GPIO12);//LED_red 灭
		//gpio_clear(GPIOC, GPIO12);
	}
#endif
}


/**
*controls for Vertical loops 
*/
#define FF_CMD_FRAC 18
static void run_hover_loop(bool in_flight)
{

  	int64_t tmp  = gv_z_ref >> (GV_Z_REF_FRAC - INT32_POS_FRAC);
  	guidance_v_z_ref = (int32_t)tmp;
  	guidance_v_zd_ref = gv_zd_ref << (INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
  	guidance_v_zdd_ref = gv_zdd_ref << (INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
 
  	int32_t err_z  = guidance_v_z_ref - stateGetPositionNed_i()->z;
	//int32_t err_z  = guidance_v_z_ref - sonar_height*256;//use the sonar
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
