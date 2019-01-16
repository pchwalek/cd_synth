#ifndef _filter_H
#define _filter_H

#include "arm_math.h"

#define NUM_STAGES	1
#define POST_SHIFT	1 // shift coefficience variables to be in range from [+1 -1) (http://www.keil.com/pack/doc/CMSIS/DSP/html/group__BiquadCascadeDF1.html#gad54c724132f6d742a444eb6df0e9c731)

void initFilter(void);
void setCutoffFreq(float inputAngle);
void applyFilter(volatile q15_t* input, volatile q15_t* output, volatile q15_t* prev_buffer);
void initFilter(void);
void adjustFilterCutoff(float desired_cutoff);
void applyCustomFilter(q15_t* input_buffer, q15_t* output_buffer, uint16_t size);
double angleToCutoffFreq(float inputAngle);
void changeQ(double new_Q);
void resetFilter();

#endif
