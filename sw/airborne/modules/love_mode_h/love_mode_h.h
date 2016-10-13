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

struct flow_stab {
  int32_t phi_p;        ///< The roll P gain on the err_vx
  int32_t phi_i;        ///< The roll I gain on the err_vx_int
  int32_t theta_p;      ///< The pitch P gain on the err_vy
  int32_t theta_i;      ///< The pitch I gain on the err_vy_int
  float desired_vx;         ///< The desired velocity in the x direction (cm/s)
  float desired_vy;         ///< The desired velocity in the y direction (cm/s)

  float err_vx_int;         ///< The integrated velocity error in x direction (m/s)
  float err_vy_int;         ///< The integrated velocity error in y direction (m/s)
  struct Int32Eulers eulers;   ///< The commands that are send to the hover loop
};
extern struct flow_stab flow_cmd;

extern void control_h_enter(void);
extern void control_h_read_rc(void);
extern void control_h_run(bool in_flight);

#endif

