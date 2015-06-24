/*
 * filter_pt1.h
 *
 *  Created on: 24 jun. 2015
 *      Author: borisb
 */


typedef float pt1_state_t;

float filter_pt1(float input, pt1_state_t* state, int f_cut);
