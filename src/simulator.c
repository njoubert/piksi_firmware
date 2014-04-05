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

#include "simulator.h"

#include <math.h>
#include <stdlib.h>

/** \simulator 
 * \{ */

simulation_state_t simulation_state = {
	.mode = 0,
	.speed = 4.0,
	.radius = 100.0,

	.pos_variance = 2.0,
	.speed_variance = 1.0,

  	.last_update_ticks = 0,
  	.current_angle_rad = 0.0,
  	.wn   = 1786,
  	.tow  = 0,
 	
 	.center_ecef = {-2700303.10144031,-4292474.39651309,3855434.34087421},
};

#define TWO_PI (M_PI*2.0)

double generateGaussianNoise(const double variance)
{
	static bool hasSpare = false;
	static double rand1, rand2;
 
	if(hasSpare)
	{
		hasSpare = false;
		return sqrt(variance * rand1) * sin(rand2);
	}
 
	hasSpare = true;
 
	rand1 = rand() / ((double) RAND_MAX);
	if(rand1 < 1e-100) rand1 = 1e-100;
	rand1 = -2 * log(rand1);
	rand2 = (rand() / ((double) RAND_MAX)) * TWO_PI;
 
	return sqrt(variance * rand1) * cos(rand2);
}

/** \} */


