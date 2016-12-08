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
#include "subsystems/datalink/telemetry.h"

#define TEST_ID 66

Debug Debug_data;
static abi_event test_ev;

static void test_agl_cb(uint8_t sender_id, float distance);

static void send_debug_data(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_MY_DEBUG(trans, dev, AC_ID,	&Debug_data.height,&Debug_data.debug0,&Debug_data.debug1);
}

void sonar_uart_init(void) 
{
	//uart6_init();
	uart_put_byte(&uart3, 0, 0x55);//want to get first data.

	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MY_DEBUG, send_debug_data);
	//AbiBindMsgAGL(TEST_ID, &test_ev, test_agl_cb);
}


void sonar_uart_periodic()
{
	uint8_t b1,b2;
	if (uart_char_available(&uart3)) 
	{
	    b2 =  uart_getch(&uart3);
		b1 =  uart_getch(&uart3);
		Debug_data.height = -(float)(265*b2+b1)/1000;
	}else uart_put_byte(&uart3, 0, 0x55);
	
	AbiSendMsgAGL(INS_INT_SONAR_ID, Debug_data.height);

}


static void test_agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
	Debug_data.height = distance;
}



