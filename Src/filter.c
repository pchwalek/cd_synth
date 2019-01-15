#include "filter.h"
#include "wave_synth.h"
#include "filter_coef.h"
#include "stdlib.h"


//#define SCALE_INPUT		22.918312
#define SCALE_INPUT		57.6140893992662

#define PI			3.1415926536
#define PI_DIV_2	1.5707963268

volatile uint8_t pastVal = -1;
volatile uint8_t inputVal = 0;
volatile uint8_t changedCutoff = 0;

float scaledAngle = 0;

arm_biquad_casd_df1_inst_q15 filter_instance;
q15_t filter_state[4] = {0};


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

// inputVal should lie somewhere between 0 and pi/2, inclusively
void setCutoffFreq(float inputAngle){

	//if angle is greater than 90-degrees (horizon), flip it
	if(inputAngle < PI_DIV_2){
		inputAngle = abs(PI - inputAngle);
	}

	// scale angle from 0 to 36 (since coefficients are calculated for 5-degree tilt increments)
	scaledAngle = SCALE_INPUT * inputAngle;

	inputVal = round(scaledAngle);


	// dont change if already set
	if(pastVal == inputVal) return;

	pastVal = inputVal;

	changedCutoff = 1;
	arm_biquad_cascade_df1_init_q15(&filter_instance, NUM_STAGES, coef[inputVal], filter_state, bit_shift[inputVal]);
}

void applyFilter(volatile q15_t* input, volatile q15_t* output, volatile q15_t* prev_buffer){
	if(changedCutoff == 1){
		arm_biquad_cascade_df1_q15(&filter_instance, prev_buffer, output, (uint32_t) BUFFER_SIZE);
		changedCutoff = 0;
	}
	arm_biquad_cascade_df1_q15(&filter_instance, input, output, (uint32_t) BUFFER_SIZE);
}
