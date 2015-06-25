/*
 * filter_pt1.c
 *
 *  Created on: 24 jun. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "common/axis.h"

#include "flight/pid.h"
#include "flight/filter.h"

uint16_t cycleTime;

// PT1 Low Pass filter
float filterApplyPt1(float input, filterState_t* state, int f_cut) {
   float dT = (float)cycleTime * 0.000001f;
   float RC= 1.0f / ( 2.0f * (float)M_PI * f_cut );

   *state = *state + dT / (RC + dT) * (input - *state);

   return *state;
}
