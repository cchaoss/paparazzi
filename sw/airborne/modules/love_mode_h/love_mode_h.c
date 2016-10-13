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
 * @file "modules/love_mode_h/love_mode_h.c"
 * @author wudong
 * @e-mail 862281335@qq.com
 * test the px4flow in ppz
 */
#include "subsystems/radio_control.h"
#include "subsystems/ins/vf_float.h"
#include "modules/love_mode_h/love_mode_h.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization.h"

#define FLOW true
#define SAFE_ANGLE_BOUND  2000 // 40 deg = 2859.1851

bool flow_flag;
struct flow_stab flow_cmd = {
  .phi_p = 15,
  .phi_i = 20,
  .theta_p = 15,
  .theta_i = 20,
  .desired_vx = 0,
  .desired_vy = 0,
  .err_vx_int = 0,
  .err_vy_int = 0,
};

void Calculate_euler(float vel_x, float vel_y, float vel_z);


void control_h_enter(void) 
{

#if FLOW == true
	  /* Reset the integrated errors */
	  flow_cmd.desired_vx = 0;
	  flow_cmd.desired_vy = 0;

	  /* Set rool/pitch to 0 degrees and psi to current heading */
	  flow_cmd.eulers.phi = 0;
	  flow_cmd.eulers.theta = 0;
	  flow_cmd.eulers.psi = stateGetNedToBodyEulers_i()->psi;
#endif
          stabilization_attitude_enter();
}

void control_h_read_rc(void)
{
	/***  guidance_h_read_rc  ***/										
	stabilization_attitude_read_rc(autopilot_in_flight, FALSE, FALSE);//GUIDANCE_H_MODE_ATTITUDE

#if FLOW == true
	if(radio_control.values[RADIO_PITCH] >1600 || radio_control.values[RADIO_PITCH] < 1400 || radio_control.values[RADIO_ROLL] > 1600 || radio_control.values[RADIO_ROLL] < 1400)
//if(USEC_OF_RC_PPM_TICKS(ppm_pulses[0])>1600||USEC_OF_RC_PPM_TICKS(ppm_pulses[0])<1400||USEC_OF_RC_PPM_TICKS(ppm_pulses[1])>1600||USEC_OF_RC_PPM_TICKS(ppm_pulses[1])<1400)
	{		
		flow_flag = false;
		flow_cmd.err_vx_int = 0;
  		flow_cmd.err_vy_int = 0;
	  	flow_cmd.desired_vx = 0;
  		flow_cmd.desired_vy = 0;

  		
  		flow_cmd.eulers.phi = 0;
  		flow_cmd.eulers.theta = 0;

	}
	else flow_flag =true;
#endif

}

void control_h_run(bool in_flight) 
{
#if FOLW == true	
	if(flow_flag)
	{
		Calculate_euler(optical_flow.flow_x, optical.flow_y,(-vff.z));
  		/* Update the setpoint */
  		stabilization_attitude_set_rpy_setpoint_i(&flow_cmd.eulers);
	}
#endif
	stabilization_attitude_run(in_flight);//GUIDANCE_H_MODE_ATTITUDE
}



/** Update the flow errors **/
void Calculate_euler(float vel_x, float vel_y, float vel_z)
{

  /* Calculate the error */ 
  float err_vx = flow_cmd.desired_vx - vel_x;
  float err_vy = flow_cmd.desired_vy - vel_y;
	BoundAbs(err_vx,150);//100cm/s

  /* Calculate the integrated errors (TODO: bound??) */
  flow_cmd.err_vx_int += err_vx / 512;
  flow_cmd.err_vy_int += err_vy / 512;
  BoundAbs(flow_cmd.err_vy_int,200);//100cm

  /* Calculate the commands */
  flow_cmd.eulers.phi = 2000;
  //flow_cmd.eulers.phi   = (flow_cmd.phi_p * err_vy + flow_cmd.phi_i * flow_cmd.err_vy_int);
  flow_cmd.eulers.theta = -(flow_cmd.theta_p * err_vx  + flow_cmd.theta_i * flow_cmd.err_vx_int);

  /* Bound the roll and pitch commands */
  BoundAbs(flow_cmd.eulers.phi, SAFE_ANGLE_BOUND);
  BoundAbs(flow_cmd.eulers.theta, SAFE_ANGLE_BOUND);
}


