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
 * @file "modules/love_mode_h/love_mode_h.h"
 * @author wudong
 * @e-mail 862281335@qq.com
 * test the px4flow in ppz
 */

#ifndef LOVE_MODE_H_H
#define LOVE_MODE_H_H

#include <std.h>
#include "math/pprz_algebra_int.h"

struct flow_stab{
int16_t pgain_x,igain_x,dgain_x;
int16_t pgain_y,igain_y,dgain_y;
float error_x;
float error_y;
float error_vx_int;
float error_vy_int;
struct Int32Eulers eulers;   ///< The commands that are send to the hover loop
};
extern struct flow_stab stab;

extern void control_h_enter(void);
extern void control_h_read_rc(void);
extern void control_h_run(bool in_flight);

#endif

