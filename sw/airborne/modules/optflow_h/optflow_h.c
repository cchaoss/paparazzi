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
 * @file "modules/optflow_h/optflow_h.c"
 * @author wudong
 * use the rc_c in v,use the flow stab in h
 */

#include "modules/optflow_h/optflow_h.h"
#include "subsystems/radio_control.h"
#include "subsystems/ins/vf_float.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "modules/optical_flow/px4flow.h"

struct flow_stab stab;

static void Calculate_euler(float vel_x, float vel_y, float vel_z);

void flow_module_init(void)
{
	stab.eulers.phi = 0;
	stab.eulers.theta = 0;
	stab.eulers.psi = stateGetNedToBodyEulers_i()->psi;
	stabilization_attitude_enter();
}
void flow_stab_run(void) 
{
	//AUX1
	if(USEC_OF_RC_PPM_TICKS(ppm_pulses[4]) > 1800)
	{
		//if(USEC_OF_RC_PPM_TICKS(ppm_pulses[0])>1600||USEC_OF_RC_PPM_TICKS(ppm_pulses[0])<1400||USEC_OF_RC_PPM_TICKS(ppm_pulses[1])>1600||USEC_OF_RC_PPM_TICKS(ppm_pulses[1])<1400)
		if(radio_control.values[RADIO_PITCH] < 1550 && radio_control.values[RADIO_PITCH] > 1450 && \
		radio_control.values[RADIO_ROLL] < 1550 && radio_control.values[RADIO_ROLL] > 1450)
		{
			Calculate_euler(optical_flow.flow_x, optical_flow.flow_y,(-vff.z));
	  		stabilization_attitude_set_rpy_setpoint_i(&stab.eulers);//Update the setpoint 
		}
		else{
			stab.eulers.phi = 0;
			stab.eulers.theta = 0;
			stab.error_vx_int = 0;
			stab.error_vy_int = 0;
			stab.error_x = 0;
			stab.error_y = 0;
		}
	}
	else{
		stab.eulers.phi = 0;
		stab.eulers.theta = 0;
		stab.error_vx_int = 0;
		stab.error_vy_int = 0;
		stab.error_x = 0;
		stab.error_y = 0;
	}
	//stabilization_attitude_run(in_flight);//GUIDANCE_H_MODE_ATTITUDE
}


#define Pmax 14000
#define Imax 5000
#define Dmax 2500
#define ANGLE_MAX  2000//40 deg = 2859.1851
uint8_t PID_Roll[3] = {150,100,80};//X
uint8_t PID_Pitch[3] = {150,100,80};//Y
static void Calculate_euler(float vel_x, float vel_y, float vel_z)
{
	/*static float old_error_x,old_error_y;
	stab.error_x -= flow.comp_x / 50;//uint:cm/s
	stab.error_y += flow.comp_y / 50;				

	stab.error_vx_int += stab.error_x;
	stab.error_vy_int += stab.error_y;
	
	stab.cmd[0] = PID_Roll[0] * stab.error_x +
					PID_Roll[1] * BoundAbs(stab.error_vx_int, -100,100) +
					PID_Roll[2] * BoundAbs((stab.error_x - old_error_x),-100,100);
	stab.cmd[1] = PID_Pitch[0] * stab.error_y +
					PID_Pitch[1] * BoundAbs(stab.error_vy_int,-100,100) +
					PID_Pitch[2] * BoundAbs((stab.error_y - old_error_y),-100,100);
	stab.cmd[0] = BoundAbs((stab.cmd[0] / 100),-130,130);
	stab.cmd[1] = BoundAbs((stab.cmd[1] / 100),-130,130);
	old_error_x = stab.error_x;
	old_error_y = stab.error_y;*/
	
	static float old_error_vx,old_error_vy;
	float error_vx = -vel_x * vel_z/2;//cm/s
	float error_vy = vel_y * vel_z/2;
	//debug[0] = error_vx;//+-100
	stab.pgain_x = PID_Roll[0] * error_vx;
	stab.pgain_y = PID_Pitch[0] * error_vy;
	BoundAbs(stab.pgain_x,Pmax);
	BoundAbs(stab.pgain_y,Pmax);

	stab.error_vx_int += error_vx / 512;
	stab.error_vy_int += error_vy / 512;
	//debug[1] = stab.error_vx_int;//+-
	stab.igain_x = PID_Roll[1] * stab.error_vx_int;
	stab.igain_y = PID_Pitch[1] * stab.error_vy_int;
	BoundAbs(stab.igain_x,Imax);
	BoundAbs(stab.igain_y,Imax);

	stab.dgain_x = PID_Roll[2] * (error_vx - old_error_vx);
	stab.dgain_y = PID_Pitch[2] * (error_vy - old_error_vy);
	//debug[2] = error_vx - old_error_vx;//+-30
	BoundAbs(stab.dgain_x,Dmax);
	BoundAbs(stab.dgain_y,Dmax);

	stab.eulers.phi = (stab.pgain_x + stab.igain_x + stab.dgain_x)/10;
	stab.eulers.phi = (stab.pgain_y + stab.igain_y +stab.dgain_y)/10;

	BoundAbs(stab.eulers.phi, ANGLE_MAX);
	BoundAbs(stab.eulers.theta,ANGLE_MAX);
	old_error_vx = error_vx;
	old_error_vy = error_vy;
	//debug[0] = stab.eulers.theta;//roll
	//debug[1] = stab.eulers.theta;//pitch
}
