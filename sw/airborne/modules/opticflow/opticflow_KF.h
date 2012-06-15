/*
 * $Id$
 *
 * Copyright (C) 2012 Hann Woei Ho <hannwoei_ho@hotmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef OF_KF
#define OF_KF

#include "std.h"
#include "math/pprz_algebra_int.h"

#define OF_STATE_SIZE 6

#ifndef OF_PRESCALER
#define OF_PRESCALER 20
#endif

/* horizontal filter propagation frequency */
#define OF_FREQ (512./OF_PRESCALER)
#define DT_HFILTER (1./OF_FREQ)

#define OF_UPDATE_SPEED

struct OFfilterInt {
  int32_t xdot;
  int32_t xdotdot;
  int32_t ydot;
  int32_t ydotdot;
  int32_t zdot;
  int32_t zdotdot;
  int32_t p;
  int32_t q;
  int32_t r;
  int32_t xP[OF_STATE_SIZE][OF_STATE_SIZE];
  uint8_t lag_counter;
  bool_t rollback;
};

extern struct OFfilterInt of_state;

extern int32_t of_dx_meas;
extern int32_t of_dy_meas;
extern int32_t of_p_meas;
extern int32_t of_q_meas;
extern int32_t of_r_meas;

extern void of_init(int32_t init_x, int32_t init_xdot, int32_t init_y, int32_t init_ydot);
extern void of_propagate(void);
extern void b2_hff_update_gps(void);
extern void of_update_opticflow(void);
extern void of_update_pos(struct Int32Vect2 pos, struct Int32Vect2 Rpos);
extern void of_update_vel(struct Int32Vect2 vel, struct Int32Vect2 Rvel);
extern void b2_hff_realign(struct FloatVect2 pos, struct FloatVect2 vel);

#define HFF_LOST_LIMIT 1000
extern uint16_t b2_hff_lost_limit;
extern uint16_t b2_hff_lost_counter;

extern void b2_hff_store_accel_body(void);

extern struct HfilterFloat *b2_hff_rb_last;
extern int lag_counter_err;
extern int save_counter;

#endif /* OF_KF */
