#include "wave_synth.h"
#include "lidar.h"
#include "tables.h"

#include "cap_touch.h"
#include "led_cntrl.h"
#include "dac.h"
#include <math.h>

#include "hall_effect.h"
#include "filter.h"
#include "arm_math.h"
#include "res_touch.h"

#include "stdlib.h"

#define MAX_INDEX 255

#define BIT_SHIFT_Q_CONV 4

#define BIT_OFFSET 127
#define DAC_FREQ 40000.0  // 50kHz
#define LIDAR_INTERPOLATE 0.0128  //  BUFFER_SIZE/DAC_FREQ

//#define BUFFER_OFFSET 512
#define SCALE_OUTPUT 4

#define FILTER_CNT 11

#define MIN_OCTAVE 3
#define MAX_OCTAVE 5

#define NOTE_C3 130.813
#define NOTE_C3S 138.591
#define NOTE_D3 146.832
#define NOTE_D3S 155.563
#define NOTE_E3 164.814
#define NOTE_F3 174.614
#define NOTE_F3S 184.997
#define NOTE_G3 195.998
#define NOTE_G3S 207.652
#define NOTE_A3 220.000
#define NOTE_A3S 207.652
#define NOTE_B3 246.942

#define NOTE_C4 261.626
#define NOTE_C4S 277.183
#define NOTE_D4 293.665
#define NOTE_D4S 311.127
#define NOTE_E4 329.628
#define NOTE_F4 349.228
#define NOTE_F4S 369.994
#define NOTE_G4 391.995
#define NOTE_G4S 415.305
#define NOTE_A4 440.000
#define NOTE_A4S 466.164
#define NOTE_B4 493.883

#define NOTE_C5 523.251
#define NOTE_C5S 554.365
#define NOTE_D5 587.330
#define NOTE_D5S 622.254
#define NOTE_E5 659.255
#define NOTE_F5 698.456
#define NOTE_F5S 739.989
#define NOTE_G5 783.991
#define NOTE_G5S 830.609
#define NOTE_A5 880.000
#define NOTE_A5S 932.328
#define NOTE_B5 987.767

#define ALPHA_DELTA_FREQ	0.005

#define UPPER_BOUND 	2048
#define BUFFER_OFFSET 	1024
#define LOWER_BOUND	0

int16_t input_val = 0;

int32_t index_1 = 0;
int32_t index_2 = 0;
int32_t index_3 = 0;
int32_t index_4 = 0;
int32_t index_5 = 0;
int32_t index_6 = 0;
int32_t index_7 = 0;
int32_t index_8 = 0;
int32_t index_9 = 0;
int32_t index_10 = 0;
int32_t index_11 = 0;
int32_t index_12 = 0;

uint8_t preWaveshape = 0;
uint8_t postWaveshape = 0;

q15_t temp[BUFFER_SIZE];

// const uint16_t *waveTable_1 = SinTable;
// const char *waveTable_2 = SawTable;
// const char *waveTable_3 = RampTable;
uint16_t* waveTable;

const uint16_t sine_wave_array[32] = {
    2047, 1648, 1264, 910,  600,  345,  156,  39,   0,    39,   156,
    345,  600,  910,  1264, 1648, 2048, 2447, 2831, 3185, 3495, 3750,
    3939, 4056, 4095, 4056, 3939, 3750, 3495, 3185, 2831, 2447};

q15_t buffer_1[BUFFER_SIZE];
q15_t buffer_2[BUFFER_SIZE];
q15_t buffer_3[BUFFER_SIZE];

q15_t filtered_buffer_1[BUFFER_SIZE];
q15_t filtered_buffer_2[BUFFER_SIZE];
q15_t filtered_buffer_3[BUFFER_SIZE];

q15_t shifted_buffer_1[BUFFER_SIZE];
q15_t shifted_buffer_2[BUFFER_SIZE];
q15_t shifted_buffer_3[BUFFER_SIZE];

uint8_t buff_toggle = 0;
int16_t max_table_index = 255;
uint8_t octave = 4;

uint8_t signal_off = 1;
// uint8_t wave = 1;

float freq_1_inc;
float freq_2_inc;
float freq_3_inc;
float freq_4_inc;
float freq_5_inc;
float freq_6_inc;
float freq_7_inc;
float freq_8_inc;
float freq_9_inc;
float freq_10_inc;
float freq_11_inc;
float freq_12_inc;

float freq_1_ind = 0;
float freq_2_ind = 0;
float freq_3_ind = 0;
float freq_4_ind = 0;
float freq_5_ind = 0;
float freq_6_ind = 0;
float freq_7_ind = 0;
float freq_8_ind = 0;
float freq_9_ind = 0;
float freq_10_ind = 0;
float freq_11_ind = 0;
float freq_12_ind = 0;

float freq_lidar_new = 0;
float freq_lidar_prev = 0;
float freq_lidar_step = 0;
float freq_lidar = 200;
float freq_lidar_inc;
float freq_lidar_ind = 0;
float temp1 = 0;
float temp2 = 0;

uint32_t prevlidarSampleTime;
uint32_t lidarSampleTime;
float time_delta;

int16_t table_val;

uint8_t lidarModeActive = 0;
uint8_t capModeActive = 0;

uint8_t temp_2 = 1;

int16_t max_filter_index = 255;
uint8_t filter_active = 0;
uint16_t* filter;
float freq_fil = 1;
float freq_fil_inc;
float freq_fil_ind = 0;
float filter_product;

float ampltiude_multiplier = 0;

float acc_vector = 0;

uint8_t setFilterIndex = 0;

uint8_t skipFilter = 0;

uint8_t IIR_filter_active = 1;

// UBaseType_t  uxSavedInterruptStatus;

void setWavetableAmplitude(uint8_t* intTracker) {
  ampltiude_multiplier = (*intTracker) / ((float)ROTATION_STEPS - 1);
  // ampltiude_multiplier = 1;
}

void activateFilter(uint8_t active) {
  if (active) {
    filter_active = 1;
    calcFilterFreq(0);
    setFilter(1);
    setFilterIndex = 0;
  } else {
    filter_active = 0;
  }
}

uint8_t isLidarModeActive(void) {
  if (lidarModeActive) return 1;
  return 0;
}
uint8_t isCapModeActive(void) {
  if (capModeActive) return 1;
  return 0;
}

void turnSoundOff(void){
  signal_off = 0;
}

void turnSoundOn(void){
  signal_off = 1;
}

void setTable(char table) {
  switch (table) {
    case 'S':
      switchTable(SinTable, sizeof(SinTable) >> 1);
      break;
    case 'W':
      switchTable(SawTable, sizeof(SawTable) >> 1);
      break;
    case 'T':
      switchTable(TriangleTable, sizeof(TriangleTable) >> 1);
      break;
    case 'R':
      switchTable(RampTable, sizeof(RampTable) >> 1);
      break;
    case 'Q':
      switchTable(SquareTable, sizeof(SquareTable) >> 1);
      break;
    default:
      switchTable(SinTable, sizeof(SinTable) >> 1);
      break;
  }
}

void setFilter(uint8_t filter) {
  switch (filter) {
    case 0:
      switchFilter(ENVELOPE2048, sizeof(ENVELOPE2048) >> 1);
      break;
    case 1:
      switchFilter(Env0, sizeof(Env0) >> 1);
      break;
    case 2:
      switchFilter(Env1, sizeof(Env1) >> 1);
      break;
    case 3:
      switchFilter(Env2, sizeof(Env2) >> 1);
      break;
    case 4:
      switchFilter(Env3, sizeof(Env3) >> 1);
      break;
//    case 5:
//      switchFilter(WAVESHAPE_SIGMOID_DATA, sizeof(WAVESHAPE_SIGMOID_DATA) >> 1);
//      break;
//    case 6:
//      switchFilter(WAVESHAPE_TANH_DATA, sizeof(WAVESHAPE_TANH_DATA) >> 1);
//      break;
//    case 7:
//      switchFilter(CHEBYSHEV_3TH_256_DATA, sizeof(CHEBYSHEV_3TH_256_DATA) >> 1);
//      break;
//    case 8:
//      switchFilter(CHEBYSHEV_4TH_256_DATA, sizeof(CHEBYSHEV_4TH_256_DATA) >> 1);
//      break;
//    case 9:
//      switchFilter(CHEBYSHEV_5TH_256_DATA, sizeof(CHEBYSHEV_5TH_256_DATA) >> 1);
//      break;
//    case 10:
//      switchFilter(CHEBYSHEV_6TH_256_DATA, sizeof(CHEBYSHEV_6TH_256_DATA) >> 1);
//      break;
    default:
      switchFilter(Env3, sizeof(Env3) >> 1);
      break;
  }
}

void incrementTable(void) {
  setFilterIndex++;
  if (setFilterIndex >= FILTER_CNT) setFilterIndex = 0;
  setFilter(setFilterIndex);
}

// uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
void turnOnLidarSounds(void) {
  if (lidarModeActive == 0) {
    lidarModeActive = 1;
    lidarStart();
    osSemaphoreRelease(lidarSampleReadySemaphoreHandle);
    // LidarMeasurement();
  }
  capModeActive = 0;
}

void turnOnCapSounds(void) {
  if (lidarModeActive) lidarStop();
  if (capModeActive != 1) {
    lidarModeActive = 0;
    capModeActive = 1;
    osSemaphoreRelease(capSampleSemaphoreHandle);
  }
  // Sample_Cap_Touch();
}

void turnOffSounds(void) {
  if (lidarModeActive) lidarStop();
  lidarModeActive = 0;
  capModeActive = 0;
}

void calcLidarFreq(int16_t* measurement) {
  prevlidarSampleTime = lidarSampleTime;
  lidarSampleTime = HAL_GetTick();
  time_delta = lidarSampleTime - prevlidarSampleTime;

  freq_lidar = freq_lidar_new;
  freq_lidar_new = 123.471 * expf(0.00288811 * ((float)*measurement));

  //freq_lidar_new = 2391.02 * log(0.401853 * ((float)*measurement));

  if (freq_lidar_new > 18000) freq_lidar_new = 18000;

  if (freq_lidar_new == freq_lidar)
    freq_lidar_step = 0;
  else {
    // linear interpolate frequency steps
    // 		find out how many approximatley how many buffers you will fill until
    // next lidar reading
//    freq_lidar_step = ((freq_lidar_new - freq_lidar) /
//                       (((time_delta / LIDAR_INTERPOLATE)) * 512));

//    freq_lidar_step = ((freq_lidar_new - freq_lidar) /
//                           (((time_delta / 40000))));
  }

  freq_lidar_inc = (freq_lidar / ((float)DAC_FREQ)) * max_table_index;
}

void updateLidarInc(void) {

  //freq_lidar += freq_lidar_step;

  freq_lidar += ALPHA_DELTA_FREQ * (freq_lidar_new - freq_lidar);
  freq_lidar_inc = (freq_lidar / ((float)DAC_FREQ)) * max_table_index;
}

void calcFilterFreqAcc(float x_g, float y_g, float z_g) {
  //	freq_fil = 123.471*expf(0.00288811*((float)*measurement));
  //	if(freq_fil > 16000) freq_fil = 16000;
  freq_fil = (sqrt(x_g * x_g + y_g * y_g + z_g * z_g));
  if (freq_fil < 1.2) {
    freq_fil = 1;
    skipFilter = 1;
  } else {
    skipFilter = 0;
  }

  freq_fil_inc = (freq_fil / ((float)DAC_FREQ)) * max_filter_index;
}

void calcFilterFreq(int16_t* measurement) {
  //	freq_fil = 123.471*expf(0.00288811*((float)*measurement));
  //	if(freq_fil > 16000) freq_fil = 16000;
  freq_fil = 0.2;

  freq_fil_inc = (freq_fil / ((float)DAC_FREQ)) * max_filter_index;
}

void DAC_BufferRefresh(void) {
  while (1) {
    // osSemaphoreWait( DAC_SemaphoreHandle, osWaitForever);
    // vTaskSuspendAll(  );
    taskENTER_CRITICAL();
    //taskDISABLE_INTERRUPTS();
    prepBuffer(&hdac1);
    //taskENABLE_INTERRUPTS();
    taskEXIT_CRITICAL();
    // xTaskResumeAll();
    vTaskSuspend(NULL);

    // taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
  }
}

void switchTable(const uint16_t* desired_table, int16_t size) {
  max_table_index = size;
  waveTable = desired_table;
  switchOctave(octave);
}

void switchFilter(const uint16_t* desired_table, int16_t size) {
  max_filter_index = size;
  filter = desired_table;
}

void prepBuffer(DAC_HandleTypeDef* hdac) {
  if (temp_2) {
    temp_2 = 0;
    switchTable(SinTable, sizeof(SinTable) >> 1);
    clearBuffer(buffer_3);
    setCutoffFreq(.9);
  }

  // determine which buffer to fill
  if (buff_toggle == 0) {
    passBufferToDAC(filtered_buffer_3, hdac);

    buff_toggle = 1;

    clearBuffer(buffer_2);
    // fill buffer depending on what button is being pressed
    fillBuffer(buffer_2);

    if (IIR_filter_active == 1) {
      // arm_float_to_q15(&temp_var, &temp_arm, 1);
      //arm_shift_q15(buffer_2, BIT_SHIFT_Q_CONV, shifted_buffer_2, BUFFER_SIZE);
      //applyFilter(shifted_buffer_2, filtered_buffer_2, shifted_buffer_3);
      if(getBitCrush() > 0){
	  applyBitCrush(buffer_2, BUFFER_SIZE);
      }

      applyCustomFilter(buffer_2, filtered_buffer_2, BUFFER_SIZE);

      if(postWaveshape > 0){
	  applyWaveshape(filtered_buffer_2, BUFFER_SIZE);
      }

//      arm_shift_q15(filtered_buffer_2, (-1 * ((int8_t)BIT_SHIFT_Q_CONV)),
//                    buffer_2, BUFFER_SIZE);
    }
  } else if (buff_toggle == 1) {
    passBufferToDAC(filtered_buffer_2, hdac);

    buff_toggle = 2;

    clearBuffer(buffer_1);
    // fill buffer depending on what button is being pressed
    fillBuffer(buffer_1);
    // addOffsetToBuffer(buffer_2);

    if (IIR_filter_active == 1) {
      // arm_float_to_q15(&temp_var, &temp_arm, 1);
      //arm_shift_q15(buffer_1, BIT_SHIFT_Q_CONV, shifted_buffer_1, BUFFER_SIZE);
      //applyFilter(shifted_buffer_1, filtered_buffer_1, shifted_buffer_2);
      if(getBitCrush() > 0){
	  applyBitCrush(buffer_1, BUFFER_SIZE);
      }

      applyCustomFilter(buffer_1, filtered_buffer_1, BUFFER_SIZE);

      if(postWaveshape > 0){
	applyWaveshape(filtered_buffer_1, BUFFER_SIZE);
      }
//      arm_shift_q15(filtered_buffer_1, (-1 * ((int8_t)BIT_SHIFT_Q_CONV)),
//                    buffer_1, BUFFER_SIZE);
    }
  } else {
    passBufferToDAC(filtered_buffer_1, hdac);

    buff_toggle = 0;

    clearBuffer(buffer_3);
    // fill buffer depending on what button is being pressed
    fillBuffer(buffer_3);
    // addOffsetToBuffer(buffer_2);
    //		float temp_var = 0.9999999999999;
    //		q15_t temp_arm = 0;
    if (IIR_filter_active == 1) {
      // arm_float_to_q15(&temp_var, &temp_arm, 1);
//      arm_shift_q15(buffer_3, BIT_SHIFT_Q_CONV, shifted_buffer_3, BUFFER_SIZE);
//      //applyFilter(shifted_buffer_3, filtered_buffer_3, shifted_buffer_1);

      if(getBitCrush() > 0){
      	  applyBitCrush(buffer_3, BUFFER_SIZE);
            }

      applyCustomFilter(buffer_3, filtered_buffer_3, BUFFER_SIZE);

      if(postWaveshape > 0){
	applyWaveshape(filtered_buffer_3, BUFFER_SIZE);
      }
//      arm_shift_q15(filtered_buffer_3, (-1 * ((int8_t)BIT_SHIFT_Q_CONV)),
//                    buffer_3, BUFFER_SIZE);
    }
  }
}
//
// void addOffsetToBuffer(q15_t* buffer){
//	for(int i = 0; i < BUFFER_SIZE; i++){
//		buffer[i] += BUFFER_OFFSET;
//	}
//}

void applyBitCrush(q15_t* buffer, uint16_t size){
  uint32_t bitCrush = getBitCrush();

  arm_shift_q15(buffer, -bitCrush, buffer, size);
  arm_shift_q15(buffer, bitCrush, buffer, size);
//  for(uint16_t i=0; i<size; i++){
//      buffer[i] = (buffer[i] & (~bitCrush) );
//  }
}

// UBaseType_t  uxSavedInterruptStatus;
BaseType_t xYieldRequired;

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
  //	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
  prepBuffer(hdac);
  //	taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
  // osSemaphoreRelease (DAC_SemaphoreHandle);
  // vTaskSuspend( NULL );
  // Resume the suspended task.
  // xYieldRequired = xTaskResumeFromISR( DAC_BufferRefreshHandle );
}

void passBufferToDAC(q15_t* buffer, DAC_HandleTypeDef* hdac) {
  HAL_GPIO_TogglePin(LED_LAT_GPIO_Port, LED_LAT_Pin);
  HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*)buffer, 512,
                    DAC_ALIGN_12B_R);
}

void clearBuffer(q15_t* buffer) {
  for (int i = 0; i < BUFFER_SIZE; i++) {
    buffer[i] = BUFFER_OFFSET;
  }
}

void fillBuffer(q15_t* buffer) {
  // if button touched, fill buffer based on its frequency
  // LED_State(KEY_1_PORT, KEY_1_PIN)

  if (lidarModeActive) {
    addTableToBuffer(buffer, &freq_lidar_inc, &freq_lidar_ind);
  } else if (capModeActive) {
    if (LED_State(CAP_1_LED_PORT, CAP_1_LED_PIN)) {
      addTableToBuffer(buffer, &freq_1_inc, &freq_1_ind);
    } else {
      reset_index(&freq_1_ind);
    }
    if (LED_State(CAP_2_LED_PORT, CAP_2_LED_PIN)) {
      addTableToBuffer(buffer, &freq_2_inc, &freq_2_ind);
    } else {
      reset_index(&freq_2_ind);
    }
    if (LED_State(CAP_3_LED_PORT, CAP_3_LED_PIN)) {
      addTableToBuffer(buffer, &freq_3_inc, &freq_3_ind);
    } else {
      reset_index(&freq_3_ind);
    }
    if (LED_State(CAP_4_LED_PORT, CAP_4_LED_PIN)) {
      addTableToBuffer(buffer, &freq_4_inc, &freq_4_ind);
    } else {
      reset_index(&freq_4_ind);
    }
    if (LED_State(CAP_5_LED_PORT, CAP_5_LED_PIN)) {
      addTableToBuffer(buffer, &freq_5_inc, &freq_5_ind);
    } else {
      reset_index(&freq_5_ind);
    }
    if (LED_State(CAP_6_LED_PORT, CAP_6_LED_PIN)) {
      addTableToBuffer(buffer, &freq_6_inc, &freq_6_ind);
    } else {
      reset_index(&freq_6_ind);
    }
    if (LED_State(CAP_7_LED_PORT, CAP_7_LED_PIN)) {
      addTableToBuffer(buffer, &freq_7_inc, &freq_7_ind);
    } else {
      reset_index(&freq_7_ind);
    }
    if (LED_State(CAP_8_LED_PORT, CAP_8_LED_PIN)) {
      addTableToBuffer(buffer, &freq_8_inc, &freq_8_ind);
    } else {
      reset_index(&freq_8_ind);
    }
    if (LED_State(CAP_9_LED_PORT, CAP_9_LED_PIN)) {
      addTableToBuffer(buffer, &freq_9_inc, &freq_9_ind);
    } else {
      reset_index(&freq_9_ind);
    }
    if (LED_State(CAP_10_LED_PORT, CAP_10_LED_PIN)) {
      addTableToBuffer(buffer, &freq_10_inc, &freq_10_ind);
    } else {
      reset_index(&freq_10_ind);
    }
    if (LED_State(CAP_11_LED_PORT, CAP_11_LED_PIN)) {
      addTableToBuffer(buffer, &freq_11_inc, &freq_11_ind);
    } else {
      reset_index(&freq_11_ind);
    }
    if (LED_State(CAP_12_LED_PORT, CAP_12_LED_PIN)) {
      addTableToBuffer(buffer, &freq_12_inc, &freq_12_ind);
    } else {
      reset_index(&freq_12_ind);
    }
  }
}

void reset_index(float* freq_ind) { freq_ind = 0; }

int32_t temp_var;
#define BIT_SMASH_AND	0x0002
#define BIT_CRUSH_DEPTH	2

void addTableToBuffer(q15_t* buffer, float* freq_inc, float* freq_ind) {
  if ((filter_active == 1) && (skipFilter == 0)) {
    if (lidarModeActive) {
      for (int i = 0; i < BUFFER_SIZE; i++) {
        updateLidarInc();
        table_val = waveTable[incrementIndex(freq_inc, freq_ind)];
        // ew... this code gets messy given that the if statement will be
        // checked at 40kHz (LOL)
        buffer[i] +=
            ampltiude_multiplier * filter_multiplier(&table_val) * SCALE_OUTPUT;
      }
    } else {
      for (int i = 0; i < BUFFER_SIZE; i++) {
        // ew... this code gets messy given that the if statement will be
        // checked at 40kHz (LOL)
        table_val = waveTable[incrementIndex(freq_inc, freq_ind)];
        buffer[i] +=
            ampltiude_multiplier * filter_multiplier(&table_val) * SCALE_OUTPUT;
      }
    }
  } else {
    if (lidarModeActive) {
      for (int i = 0; i < BUFFER_SIZE; i++) {
        updateLidarInc();
        table_val = waveTable[incrementIndex(freq_inc, freq_ind)];

        if(preWaveshape > 0){
            if(preWaveshape == 1) table_val = WAVESHAPE_CHEBYSHEV_4TH_256_DATATANH_DATA[table_val+127];
            else if(preWaveshape == 2) table_val = WAVESHAPE_SIGMOID_DATA[table_val+127];
            else if(preWaveshape == 3) table_val = WAVESHAPE_TANH_DATA[table_val+127];
        }

        buffer[i] += ( ( (int32_t) (ampltiude_multiplier * (table_val * SCALE_OUTPUT) ) )) * signal_off;
//
//	temp_var = (int32_t) (ampltiude_multiplier * table_val * SCALE_OUTPUT);

        //uint16_t randomInt = rand() % 2;

//        if(randomInt == 1){
//            //buffer[i] +=  ( (int32_t) (ampltiude_multiplier * table_val * SCALE_OUTPUT) ) & (~BIT_SMASH_AND);
//
//        }
//        else
//          {
//            //buffer[i] +=  ( (int32_t) (ampltiude_multiplier * table_val * SCALE_OUTPUT) ) | (BIT_SMASH_AND );
//          }
        //buffer[i] += ampltiude_multiplier * SCALE_OUTPUT * ( ( ( (int32_t) (table_val) ) >> 2 ) << 2 );

      }
    } else {
      for (int i = 0; i < BUFFER_SIZE; i++) {
        table_val = waveTable[incrementIndex(freq_inc, freq_ind)];

//        table_val = table_val[WAVESHAPE_CHEBYSHEV_4TH_256_DATATANH_DATA+127];
        //buffer[i] += ampltiude_multiplier * table_val * SCALE_OUTPUT;
        buffer[i] += ( ( (int32_t) (ampltiude_multiplier * (table_val * SCALE_OUTPUT) ) )) * signal_off;
      }
    }
  }
}

q15_t filter_multiplier(int16_t* waveTable) {
  filter_product =
      (*waveTable) *
      (filter[incrementFilterIndex(&freq_fil_inc, &freq_fil_ind)] / 255.0);
  return (q15_t)filter_product;
}

uint16_t incrementIndex(float* freq_inc, float* freq_ind) {
  *freq_ind += (*freq_inc);
  if ((*freq_ind) >= max_table_index) {
    (*freq_ind) -= max_table_index;
  }

  return (uint16_t)(*freq_ind);
}

uint16_t incrementFilterIndex(float* freq_inc, float* freq_ind) {
  *freq_ind += (*freq_inc);
  if ((*freq_ind) >= max_filter_index) {
    (*freq_ind) -= max_filter_index;
  }

  return (uint16_t)(*freq_ind);
}

void switchOctave(uint8_t des_octave) {
  octave = des_octave;
  if (des_octave == 4) {
    freq_1_inc = (NOTE_C4 / DAC_FREQ) * max_table_index;
    freq_2_inc = (NOTE_C4S / DAC_FREQ) * max_table_index;
    freq_3_inc = (NOTE_D4 / DAC_FREQ) * max_table_index;
    freq_4_inc = (NOTE_D4S / DAC_FREQ) * max_table_index;
    freq_5_inc = (NOTE_E4 / DAC_FREQ) * max_table_index;
    freq_6_inc = (NOTE_F4 / DAC_FREQ) * max_table_index;
    freq_7_inc = (NOTE_F4S / DAC_FREQ) * max_table_index;
    freq_8_inc = (NOTE_G4 / DAC_FREQ) * max_table_index;
    freq_9_inc = (NOTE_G4S / DAC_FREQ) * max_table_index;
    freq_10_inc = (NOTE_A4 / DAC_FREQ) * max_table_index;
    freq_11_inc = (NOTE_A4S / DAC_FREQ) * max_table_index;
    freq_12_inc = (NOTE_B4 / DAC_FREQ) * max_table_index;
  } else if (des_octave == 5) {
    freq_1_inc = (NOTE_C5 / DAC_FREQ) * max_table_index;
    freq_2_inc = (NOTE_C5S / DAC_FREQ) * max_table_index;
    freq_3_inc = (NOTE_D5 / DAC_FREQ) * max_table_index;
    freq_4_inc = (NOTE_D5S / DAC_FREQ) * max_table_index;
    freq_5_inc = (NOTE_E5 / DAC_FREQ) * max_table_index;
    freq_6_inc = (NOTE_F5 / DAC_FREQ) * max_table_index;
    freq_7_inc = (NOTE_F5S / DAC_FREQ) * max_table_index;
    freq_8_inc = (NOTE_G5 / DAC_FREQ) * max_table_index;
    freq_9_inc = (NOTE_G5S / DAC_FREQ) * max_table_index;
    freq_10_inc = (NOTE_A5 / DAC_FREQ) * max_table_index;
    freq_11_inc = (NOTE_A5S / DAC_FREQ) * max_table_index;
    freq_12_inc = (NOTE_B5 / DAC_FREQ) * max_table_index;
  } else if (des_octave == 3) {
    freq_1_inc = (NOTE_C3 / DAC_FREQ) * max_table_index;
    freq_2_inc = (NOTE_C3S / DAC_FREQ) * max_table_index;
    freq_3_inc = (NOTE_D3 / DAC_FREQ) * max_table_index;
    freq_4_inc = (NOTE_D3S / DAC_FREQ) * max_table_index;
    freq_5_inc = (NOTE_E3 / DAC_FREQ) * max_table_index;
    freq_6_inc = (NOTE_F3 / DAC_FREQ) * max_table_index;
    freq_7_inc = (NOTE_F3S / DAC_FREQ) * max_table_index;
    freq_8_inc = (NOTE_G3 / DAC_FREQ) * max_table_index;
    freq_9_inc = (NOTE_G3S / DAC_FREQ) * max_table_index;
    freq_10_inc = (NOTE_A3 / DAC_FREQ) * max_table_index;
    freq_11_inc = (NOTE_A3S / DAC_FREQ) * max_table_index;
    freq_12_inc = (NOTE_B3 / DAC_FREQ) * max_table_index;
  }
}

void incrementOctave(void) {
  if (octave == MAX_OCTAVE)
    return;
  else {
    octave++;
    switchOctave(octave);
  }
}

void decrementOctave(void) {
  if (octave == MIN_OCTAVE)
    return;
  else {
    octave--;
    switchOctave(octave);
  }
}

void setPreWave(uint8_t desWave){
  preWaveshape = desWave;
}

void setPostWave(uint8_t desWave){
  preWaveshape = desWave;
}

void applyWaveshape(q15_t* buffer, uint16_t size){

  uint8_t index;

  for(uint16_t i = 0; i<size; i++){
    if(buffer[i] > UPPER_BOUND) buffer[i] == UPPER_BOUND;
    else if (buffer[i] < LOWER_BOUND) buffer[i] == LOWER_BOUND;

    index = round(255 * (buffer[i] - LOWER_BOUND) / ((float) UPPER_BOUND - LOWER_BOUND));

    if(postWaveshape == 1) buffer[i] = WAVESHAPE_CHEBYSHEV_4TH_256_DATATANH_DATA[index];
    else if(postWaveshape == 2) buffer[i] = WAVESHAPE_SIGMOID_DATA[index];
    else if(postWaveshape == 3) buffer[i] = WAVESHAPE_TANH_DATA[index];
  }
}

