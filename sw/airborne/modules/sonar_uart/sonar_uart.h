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
 * @file "modules/sonar_uart/sonar_uart.h"
 * @author WuDong
 * 
 */

#ifndef SONAR_UART_H
#define SONAR_UART_H

typedef struct _debug{
uint16_t debug0;
uint32_t debug1;
float	height;
}Debug;
extern Debug Debug_data;

extern void sonar_uart_init(void);
extern void sonar_uart_periodic(void);

#endif

