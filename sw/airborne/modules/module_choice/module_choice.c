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
 * @file "modules/module_choice/module_choice.c"
 * @author wudong
 * 
 */

#include "led.h"
#include "subsystems/ins/vf_float.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/module_choice/module_choice.h"
#include "subsystems/radio_control.h"
#include "subsystems/electrical.h"

char module_number;

void module_choice_init(void) 
{
	module_number = 0;

	/*************************    判断气压计数据是否正常    SERVO_5 --> LED_yellow   *************************/
/*
	if(vff.z == 0)		gpio_setup_input(GPIOA, GPIO0);//气压计无数据-->LED_red 灭
	else
	{
		module_number = 0;
		autopilot_set_motors_on(FALSE);
		//gpio_setup_output(GPIOA, GPIO0);//气压计数据正常-->LED_yellow 亮
		//gpio_set(GPIOA, GPIO0);
	}
*/
}


void module_choice_periodic(void) 
{




	/*************************    判断飞控是否解锁    ****************************************************/
	/*if(autopilot_motors_on == FALSE)	gpio_setup_input(GPIOC, GPIO12);//飞控没有解锁 LED_red 灭
	else
	{		
		gpio_setup_output(GPIOC, GPIO12);//解锁后 LED_red 亮
		gpio_set(GPIOC, GPIO12);
	}	
	*/


	/*************************    判断遥控信号   *****************************************************/
	if(radio_control.status == RC_OK)
	{
		if (USEC_OF_RC_PPM_TICKS(ppm_pulses[5]) < 1200)		
		{
			module_number = 1;	//autopilot_motors_on = TRUE;
		}
		//else if (USEC_OF_RC_PPM_TICKS(ppm_pulses[6])>1200 && USEC_OF_RC_PPM_TICKS(ppm_pulses[6])<1800)	module_number = 2;

		else 
		{	
			module_number = 3;	//autopilot_motors_on = FALSE;

		}
	}
	else 
	{	
		module_number = 0;

	}
	//you can add more choicees,


}

