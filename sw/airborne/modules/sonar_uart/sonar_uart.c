/*
 * Copyright (C) WuDong
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
 * @file "modules/sonar_uart/sonar_uart.c"
 * @author WuDong
 * 
 */

#include "mcu_periph/uart.h"
#include "modules/sonar_uart/sonar_uart.h"
#include "subsystems/abi_common.h"
#include "subsystems/abi.h"

#define TEST_ID 66

float height_sonar,debug1,debug2,debug3;

static abi_event test_ev;



static void test_agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
	height_sonar = distance;
}

void sonar_uart_init(void) 
{
	//	uart6_init();
	//uart_put_byte(&uart3, 0, 0x55);
	AbiBindMsgAGL(TEST_ID, &test_ev, test_agl_cb);
}


void sonar_uart_periodic()
{/*
	static a = 0;
	uint8_t b,c;
	if (uart_char_available(&uart3)) 
	{
	    c =  uart_getch(&uart3);
		b =  uart_getch(&uart3);
		d = -(float)(265*c+b)/1000;
	}else uart_put_byte(&uart3, 0, 0x55);
	
	if(fabs(d) > 4)
	{
		a++;
		if(a > 3) d = -1.0;
	}else a = 0;
	*/
	static float d = 0;
	d++;
	if(d > 65535)d= 0;
	AbiSendMsgAGL(TEST_ID, d);
#if 0
	if (uart_char_available(&uart6)) 
	{
	    uint8_t c =  uart_getch(&uart6);
		uart_put_byte(&uart6, 0, c);
	}
	else uart_put_byte(&uart4, 0, 0x11);
	//uart_put_byte(&uart6, 0, 0x11);
#endif
}


