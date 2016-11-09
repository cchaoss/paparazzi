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
 * @file modules/ctrl/vertical_ctrl_module_demo.c
 * @brief example vertical controller
 *
 */

#include "modules/sonar_uart/sonar_uart.h"
#include "modules/ctrl/vertical_ctrl_module_demo.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"

/* Default sonar/agl to use */
#ifndef VERTICAL_CTRL_MODULE_AGL_ID
#define VERTICAL_CTRL_MODULE_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(VERTICAL_CTRL_MODULE_AGL_ID)

#ifndef VERTICAL_CTRL_MODULE_PGAIN
#define VERTICAL_CTRL_MODULE_PGAIN 2000
#endif

#ifndef VERTICAL_CTRL_MODULE_IGAIN
#define VERTICAL_CTRL_MODULE_IGAIN 0.03
#endif

struct VerticalCtrlDemo v_ctrl;


void vertical_ctrl_module_init(void);
void vertical_ctrl_module_run(bool in_flight);

void vertical_ctrl_module_init(void)
{
  v_ctrl.agl = 0.0f;
  v_ctrl.setpoint = 1.0f;
  v_ctrl.pgain = VERTICAL_CTRL_MODULE_PGAIN;
  v_ctrl.igain = VERTICAL_CTRL_MODULE_IGAIN;
  v_ctrl.sum_err = 0.0f;

}


void vertical_ctrl_module_run(bool in_flight)
{
  	if (!in_flight) 
	{
    	// Reset integrators
    	v_ctrl.sum_err = 0;
    	stabilization_cmd[COMMAND_THRUST] = 0;
  	} 
	else 
	{
		v_ctrl.agl = fabs(height_sonar);
		//int32_t nominal_throttle = 0.5 * MAX_PPRZ;
		float err = v_ctrl.setpoint - v_ctrl.agl;	debug1 = err;//
		int16_t P = v_ctrl.pgain * err*10;
		Bound(P, -800, 800);
		int16_t I = v_ctrl.igain * v_ctrl.sum_err >> 9;  debug2 = v_ctrl.sum_err;//
		Bound(I, -100, 100);
		int16_t D = v_ctrl.dgain * (error - error_old)/512; debug3 = (float)D;//
		Bound(D, -300,300);
		int16_t thrust = 4800 + P + I;Bound(thrust, 0, MAX_PPRZ);	//debug4 = (float)thrust;//
		
		stabilization_cmd[COMMAND_THRUST] = thrust;
		v_ctrl.sum_err += err;	
		static float error_old = error;

  	}
}


////////////////////////////////////////////////////////////////////
// Call our controller
void guidance_v_module_init(void)
{
  vertical_ctrl_module_init();
}

void guidance_v_module_enter(void)
{
  // reset integrator
  v_ctrl.setpoint = fabs(height_sonar);
  v_ctrl.sum_err = 0.0f;
}

void guidance_v_module_run(bool in_flight)
{
  vertical_ctrl_module_run(in_flight);
}
