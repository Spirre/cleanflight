/*
 * filter_pt1.h
 *
 *  Created on: 24 jun. 2015
 *      Author: borisb
 */


typedef float filterState_t;

float filterApplyPt1(float input, filterState_t* state, int f_cut);
