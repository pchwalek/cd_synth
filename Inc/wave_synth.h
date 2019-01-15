#ifndef __wave_synth_H
#define __wave_synth_H
#include "arm_math.h"
#include "stdint.h"
#include "dac.h"
#include "cmsis_os.h"

#define BUFFER_SIZE 512

osSemaphoreId DAC_SemaphoreHandle;
osThreadId DAC_BufferRefreshHandle;

void setWavetableAmplitude(volatile uint8_t* intTracker);
void incrementOctave(void);
void decrementOctave(void);
void setTable(char table);
void DAC_BufferRefresh(void);
void switchOctave(uint8_t des_octave);
void playSample(void);
void clearBuffer(volatile q15_t* buffer);
void fillBuffer(volatile q15_t* buffer);
void prepBuffer(DAC_HandleTypeDef* hdac);
void switchTable(volatile uint16_t* desired_table, volatile int16_t size);
void reset_index(volatile float* freq_ind);
void addTableToBuffer(volatile q15_t* buffer, volatile float* freq_inc, volatile float* freq_ind);
uint16_t incrementIndex(volatile float* freq_inc, volatile float* freq_ind);
void passBufferToDAC(volatile q15_t* buffer, DAC_HandleTypeDef* hdac);
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void calcLidarFreq(int16_t* measurement);
void calcFilterFreqAcc(float x_g, float y_g, float z_g);
void calcFilterFreq(int16_t* measurement);
void updateLidarInc(void);
q15_t filter_multiplier(int16_t* waveTable);
uint16_t incrementFilterIndex(volatile float* freq_inc, volatile float* freq_ind);
void switchFilter(volatile uint16_t* desired_table, volatile int16_t size);
void setFilter(uint8_t filter);
void incrementTable(void);
void activateFilter(uint8_t active);

uint8_t isLidarModeActive(void);
uint8_t isCapModeActive(void);

void turnOnLidarSounds(void);
void turnOnCapSounds(void);
void turnOffSounds(void);

#endif
