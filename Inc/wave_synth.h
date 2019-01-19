#ifndef __wave_synth_H
#define __wave_synth_H
#include "arm_math.h"
#include "stdint.h"
#include "dac.h"
#include "cmsis_os.h"

#define BUFFER_SIZE 512

osSemaphoreId DAC_SemaphoreHandle;
osSemaphoreId bufferFillSemaphoreHandle;
osThreadId DAC_BufferRefreshHandle;

void setWavetableAmplitude(uint8_t* intTracker);
void incrementOctave(void);
void decrementOctave(void);
void setTable(char table);
void DAC_BufferRefresh(void);
void switchOctave(uint8_t des_octave);
void playSample(void);
void clearBuffer(q15_t* buffer);
void fillBuffer(q15_t* buffer);
void prepBuffer(void);
void switchTable(const uint16_t* desired_table, int16_t size);
void reset_index(float* freq_ind);
void addTableToBuffer(q15_t* buffer, float* freq_inc, float* freq_ind);
uint16_t incrementIndex(float* freq_inc, float* freq_ind);
void passBufferToDAC(q15_t* buffer);
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void calcLidarFreq(int16_t* measurement);
void calcFilterFreqAcc(float x_g, float y_g, float z_g);
void calcFilterFreq(int16_t* measurement);
void updateLidarInc(void);
q15_t filter_multiplier(int16_t* waveTable);
uint16_t incrementFilterIndex(float* freq_inc, float* freq_ind);
void switchFilter(const uint16_t* desired_table, int16_t size);
void setFilter(uint8_t filter);
void incrementTable(void);
void activateFilter(uint8_t active);
void turnSoundOn(void);
void turnSoundOff(void);
void applyBitCrush(q15_t* buffer, uint16_t size);
void setPreWave(uint8_t desWave);
void setPostWave(uint8_t desWave);
void applyWaveshape(q15_t* buffer, uint16_t size);
void activateLowpassFilter(uint8_t activate);

uint8_t isLidarModeActive(void);
uint8_t isCapModeActive(void);

void turnOnLidarSounds(void);
void turnOnCapSounds(void);
void turnOffSounds(void);

#endif
