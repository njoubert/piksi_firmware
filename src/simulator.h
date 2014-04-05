/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SIMULATOR_H
#define SWIFTNAV_SIMULATOR_H

#include <libswiftnav/common.h>

/** \addtogroup simulator
 * \{ */

typedef struct {
  u8      mode;     //0 == off, 1 == PVT only, 2 == PVT & Baseline
  float   speed;    //speed (variance of velocity) in meters per second
  float   radius;   //radius of circle in meters

  float pos_variance;     //in meters squared
  float speed_variance;   //variance in speed (magnitude of velocity) in meters squared

  u32     last_update_ticks;
  float  current_angle_rad;
  u16     wn;
  u32     tow;
  
  double  center_ecef[3];
} simulation_state_t;

/** \} */

extern simulation_state_t simulation_state;

double generateGaussianNoise(const double variance);

#endif  /* SWIFTNAV_SIMULATOR_H */

