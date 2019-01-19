#include "filter.h"
#include "wave_synth.h"
#include "filter_coef.h"
#include "stdlib.h"

//#define APPLY_SOFT_CLIPPING 1
#define CENTER_AMPLITUDE    1024
#define MAX_FREQ	    2048

//#define SCALE_INPUT		22.918312
#define SCALE_INPUT		57.6140893992662

//#define PI			3.1415926536
#define PI_DIV_2	1.5707963268

#define MAX_CUTOFF_FREQ	16000
#define SAMPLING_FREQ	40000

volatile float pastVal = -1;
volatile uint8_t inputVal = 0;
volatile uint8_t changedCutoff = 0;

float scaledAngle = 0;

arm_biquad_casd_df1_inst_q15 filter_instance;
q15_t filter_state[4] = {0};

double Q = .4;
uint16_t x_2 = 0;
uint16_t x_1 = 0;
uint16_t y_2 = 0;
uint16_t y_1 = 0;

float b_0 = 0;
float b_1 = 0;
float b_2 = 0;
float a_0 = 0;
float a_1 = 0;
float a_2 = 0;

double omega = 0;
double alpha = 0;

float current_cutoff = 16000;

float cos_omega = 0;
float sin_omega = 0;

double new_cutoff = 16000;


//q15_t coef_1000[6*NUM_STAGES] = {0.005709266664235, 0, 0.011418533328471, 0.005709266664235, 1.832076711084677, -0.854913777741618};
//q15_t coef_1000[6*NUM_STAGES] = {32767, 0, 0, 0, 0, 0};
//const q15_t coef_1000[] = {93.5, 0, 187.0, 93.5, 30016.5, -14006.5};
//const q15_t coef_1000[] = {24.0, 0, 48.5, 24.0, 31433.5, -15147.0};

void initFilter(void){
	filter_instance.pCoeffs = coef[FILTER_VALS-1];
	filter_instance.postShift = bit_shift[FILTER_VALS-1];
	filter_instance.numStages = NUM_STAGES;
	filter_instance.pState = filter_state;

	arm_biquad_cascade_df1_init_q15(&filter_instance, NUM_STAGES, coef[FILTER_VALS-1], filter_state, bit_shift[FILTER_VALS-1]);

}

// ref: http://shepazu.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
void adjustFilterCutoff(float desired_cutoff){
  if(desired_cutoff > MAX_CUTOFF_FREQ){
      desired_cutoff = MAX_CUTOFF_FREQ;
  }

  if(desired_cutoff == current_cutoff){
        return;
    }

  if(desired_cutoff >= 0){
      current_cutoff = desired_cutoff;
  }

  // calculate omega_not
  omega = 2 * PI * current_cutoff / ((float) SAMPLING_FREQ);

  // precalculate sin and cos of omega
  cos_omega = cos(omega);
  sin_omega = sin(omega);

  // calculate alpha
  alpha = sin_omega/ (2.0*Q);

  // calculate coefficients and normalize by a_0
  a_0 = 1 + alpha;

  b_0 = ((1.0 - cos_omega) / 2.0) / a_0;
  b_1 = (1.0 - cos_omega) / a_0;
  b_2 = b_0;
  a_1 = (-2.0 * cos_omega) / a_0;
  a_2 = (1.0 - alpha) / a_0;
}

uint32_t startT;
uint32_t totalT;
q15_t temp_buffer[512];
q15_t tempVal;

void applyCustomFilter(q15_t* input_buffer, q15_t* output_buffer, uint16_t size){
//  startT = DWT->CYCCNT;
  for(uint16_t i = 0; i < size; i++){
      output_buffer[i] = b_0 * input_buffer[i]
		      + b_1 * x_1
		      + b_2 * x_2
		      - a_1 * y_1
		      - a_2 * y_2;


//      arm_float_to_q15(&b_0, &tempVal, 1);
//
//
//      arm_scale_q15(input_buffer,tempVal,0,temp_buffer,512);

      //ref: https://www.hackaudio.com/digital-signal-processing/distortion-effects/soft-clipping/
#ifdef APPLY_SOFT_CLIPPING
      //output_buffer[i] = 2048 * (1/1+exp((1000-output_buffer[i])/1024));
      //output_buffer[i] = 1024 * tanh( (output_buffer[i]/1024.0) - 1 ) + 1024;
      //output_buffer[i] = output_buffer[i] - (1/3) * (output_buffer[i]*output_buffer[i]*output_buffer[i]);
      output_buffer[i] = 1024.0*(2.0/PI)*atan((output_buffer[i]-1024.0)/700.0) + 1024.0;

#endif

      x_2 = x_1;
      x_1 = input_buffer[i];
      y_2 = y_1;
      y_1 = output_buffer[i];
  }
//  totalT = DWT->CYCCNT - startT;
//  startT = DWT->CYCCNT - totalT;
}

void resetFilterHistory(void){
  x_2 = 0;
  x_1 = 0;
  y_2 = 0;
  y_1 = 0;
}

// inputVal should lie somewhere between 0 and pi/2, inclusively
void setCutoffFreq(float inputAngle){

	//if angle is greater than 90-degrees (horizon), flip it
//	if(inputAngle < PI_DIV_2){
//		inputAngle = abs(PI - inputAngle);
//	}

	// scale angle from 0 to 36 (since coefficients are calculated for 5-degree tilt increments)
//	scaledAngle = SCALE_INPUT * inputAngle;
//
//	inputVal = round(scaledAngle);


	// dont change if already set
	if(pastVal == inputAngle) return;

	pastVal = inputAngle;

//	changedCutoff = 1;
//	arm_biquad_cascade_df1_init_q15(&filter_instance, NUM_STAGES, coef[inputVal], filter_state, bit_shift[inputVal]);

	new_cutoff = angleToCutoffFreq(inputAngle);
	adjustFilterCutoff(new_cutoff);
}

double angleToCutoffFreq(float inputAngle){
  return 16000 * expf(-1.01966 * inputAngle);
}

void changeQ(double new_Q){
  Q = new_Q;
  adjustFilterCutoff(-1);
}

void applyFilter(volatile q15_t* input, volatile q15_t* output, volatile q15_t* prev_buffer){
	if(changedCutoff == 1){
		arm_biquad_cascade_df1_q15(&filter_instance, prev_buffer, output, (uint32_t) BUFFER_SIZE);
		changedCutoff = 0;
	}
	arm_biquad_cascade_df1_q15(&filter_instance, input, output, (uint32_t) BUFFER_SIZE);
}
