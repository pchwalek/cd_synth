#include "led_cntrl.h"
//#include "stm32l4xx_hal_gpio.h"
//#include "spi.h"
//#include "main.h"
#include "spi.h"
//#include "task.h"
#include "res_touch.h"
#include "string.h"
#include "math.h"
#include "tim.h"
#include "stdlib.h"
#include "stm32l4xx_hal_uart.h"
#include "usart.h"
#include "hall_effect.h"
#include "lidar.h"

#define HALF_MAGNETS 2

#define usTIMER_PERIOD 9
#define TIMER_SCALAR 9  // usTimer_Period * 10^x where x is...

#define POV_RESOLUTION 32

// when to trigger POV display -> setting to 1s half spin
//		setting: 1s -> x/80MHz = 1s -> x = 80000000
//#define TICK_POV_THRESH 20000000
#define TICK_POV_THRESH 30000000

#define TICK_POV_MICROSEC_10_DIVIDER 800

uint32_t prevTracker = 0;
int64_t deltaTracker = 0;

int64_t trigDeltaTracker = 0;

double delayCycles = 0;

uint32_t switchPOV = 0;
uint32_t timerPOVstate = 0;

uint32_t usTickTracker = 0;

uint32_t indexTracker = 0;

uint8_t POV_timerActive = 0;

uint32_t tick_threshold = 0;
uint32_t hwTriggerCnt = 0;
uint8_t firstRun = 1;

uint32_t lidarPOV_Map = 0;

uint32_t constantRPR = 10000000;
uint16_t global_visNum = 0;

//#pragma GCC push_options
//#pragma GCC optimize ("O3")
// void delayUS_DWT(uint32_t us) {
//	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
//	volatile uint32_t start = DWT->CYCCNT;
//	do  {
//	} while(DWT->CYCCNT - start < cycles);
//}
//#pragma GCC pop_options

// const uint8_t message_resenv_green[MESSAGE_LENGTH][MESSAGE_WIDTH] = {
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 0, 0, 1, 1, 0},
//		{1, 0, 0, 0, 1, 1, 0},
//		{0, 0, 1, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 1, 1, 0, 1, 1, 0},
//		{0, 1, 1, 0, 1, 1, 0},
//		{0, 1, 1, 0, 1, 1, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 1, 1, 0, 0, 0, 0},
//		{0, 1, 1, 0, 1, 1, 0},
//		{0, 1, 1, 0, 1, 1, 0},
//		{0, 0, 0, 0, 1, 1, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1}
//};
//
// const uint8_t message_resenv_red[MESSAGE_LENGTH][MESSAGE_WIDTH] = {
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{1, 0, 0, 1, 0, 0, 1},
//		{1, 0, 0, 1, 0, 0, 1},
//		{1, 0, 0, 1, 0, 0, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 1, 1, 0},
//		{0, 1, 1, 1, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 1, 1, 1, 1, 1},
//		{1, 1, 0, 0, 0, 0, 0},
//		{1, 1, 0, 0, 0, 0, 0},
//		{0, 0, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0}
//};

const uint8_t message_resenv_green[MESSAGE_LENGTH][MESSAGE_WIDTH] = {
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 1, 0, 1}, {0, 0, 1, 1, 0, 1, 1},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 1, 1, 1, 1, 1}, {0, 0, 1, 0, 1, 0, 1},
    {0, 0, 1, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 1, 0, 1, 1, 1},
    {0, 0, 1, 0, 1, 0, 1}, {0, 0, 1, 1, 1, 0, 1}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}};

const uint8_t message_resenv_red[MESSAGE_LENGTH][MESSAGE_WIDTH] = {
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 1, 1, 1, 1, 1},
    {0, 0, 1, 0, 1, 0, 1}, {0, 0, 1, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 1, 0, 0},
    {0, 0, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 1, 1, 1},
    {0, 0, 1, 1, 0, 0, 0}, {0, 0, 0, 0, 1, 1, 1}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}};

uint8_t lidar_green[MESSAGE_LENGTH][MESSAGE_WIDTH] = {
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}};

uint8_t lidar_red[MESSAGE_LENGTH][MESSAGE_WIDTH] = {
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}};

// const uint8_t message_resenv_green[MESSAGE_LENGTH][MESSAGE_WIDTH] = {
//
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//};
//
//
//
//
//
//
// const uint8_t message_resenv_red[MESSAGE_LENGTH][MESSAGE_WIDTH] = {
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 1, 1, 1},
//		{0, 0, 0, 0, 0, 0, 0},
//};

void POV_handler(uint64_t RPR) {
  switchPOV++;

  if ((RPR) <= ((uint32_t)TICK_POV_THRESH)) {

    // disable any functions that use LED display
    disable_buttons();

    usTickTracker = (uint32_t)RPR;
    // deltaTracker = deltaTracker / HALF_MAGNETS;
    visualizationRun(2, usTickTracker);
  }
}

// POV_left() modifies map for left pixels
// POV_right() modifies map for right pixels
void POV_left(uint8_t* colorMap, uint8_t color) {
  // set to red
  if (color == 1) {
    if (colorMap[0] == 1) {
      LED_SETTINGS[POV_4_R_REG] &= ~POV_4_R_PIN;
    }

    if (colorMap[1] == 1) {
      LED_SETTINGS[BUTTON_3_R_REG] &= ~BUTTON_3_R_PIN;
    }

    if (colorMap[2] == 1) {
      LED_SETTINGS[POV_3_R_REG] &= ~POV_3_R_PIN;
    }

    if (colorMap[3] == 1) {
      LED_SETTINGS[BUTTON_2_R_REG] &= ~BUTTON_2_R_PIN;
    }

    if (colorMap[4] == 1) {
      LED_SETTINGS[POV_2_R_REG] &= ~POV_2_R_PIN;
    }

    if (colorMap[5] == 1) {
      LED_SETTINGS[BUTTON_1_R_REG] &= ~BUTTON_1_R_PIN;
    }

    if (colorMap[6] == 1) {
      LED_SETTINGS[POV_1_R_REG] &= ~POV_1_R_PIN;
    }

  }

  // set to green
  else {
    if (colorMap[0] == 1) {
      LED_SETTINGS[POV_4_G_REG] &= ~POV_4_G_PIN;
    }

    if (colorMap[1] == 1) {
      LED_SETTINGS[BUTTON_3_G_REG] &= ~BUTTON_3_G_PIN;
    }

    if (colorMap[2] == 1) {
      LED_SETTINGS[POV_3_G_REG] &= ~POV_3_G_PIN;
    }

    if (colorMap[3] == 1) {
      LED_SETTINGS[BUTTON_2_G_REG] &= ~BUTTON_2_G_PIN;
    }

    if (colorMap[4] == 1) {
      LED_SETTINGS[POV_2_G_REG] &= ~POV_2_G_PIN;
    }

    if (colorMap[5] == 1) {
      LED_SETTINGS[BUTTON_1_G_REG] &= ~BUTTON_1_G_PIN;
    }

    if (colorMap[6] == 1) {
      LED_SETTINGS[POV_1_G_REG] &= ~POV_1_G_PIN;
    }
  }
}

void POV_right(uint8_t* colorMap, uint8_t color) {
  // set to red
  if (color == 1) {
    if (colorMap[0] == 1) {
      LED_SETTINGS[POV_5_R_REG] &= ~POV_5_R_PIN;
    }

    if (colorMap[1] == 1) {
      LED_SETTINGS[BUTTON_4_R_REG] &= ~BUTTON_4_R_PIN;
    }

    if (colorMap[2] == 1) {
      LED_SETTINGS[POV_6_R_REG] &= ~POV_6_R_PIN;
    }

    if (colorMap[3] == 1) {
      LED_SETTINGS[BUTTON_5_R_REG] &= ~BUTTON_5_R_PIN;
    }

    if (colorMap[4] == 1) {
      LED_SETTINGS[POV_7_R_REG] &= ~POV_7_R_PIN;
    }

    if (colorMap[5] == 1) {
      LED_SETTINGS[BUTTON_6_R_REG] &= ~BUTTON_6_R_PIN;
    }

    if (colorMap[6] == 1) {
      LED_SETTINGS[POV_8_R_REG] &= ~POV_8_R_PIN;
    }

  }

  // set to green
  else {
    if (colorMap[0] == 1) {
      LED_SETTINGS[POV_5_G_REG] &= ~POV_5_G_PIN;
    }

    if (colorMap[1] == 1) {
      LED_SETTINGS[BUTTON_4_G_REG] &= ~BUTTON_4_G_PIN;
    }

    if (colorMap[2] == 1) {
      LED_SETTINGS[POV_6_G_REG] &= ~POV_6_G_PIN;
    }

    if (colorMap[3] == 1) {
      LED_SETTINGS[BUTTON_5_G_REG] &= ~BUTTON_5_G_PIN;
    }

    if (colorMap[4] == 1) {
      LED_SETTINGS[POV_7_G_REG] &= ~POV_7_G_PIN;
    }

    if (colorMap[5] == 1) {
      LED_SETTINGS[BUTTON_6_G_REG] &= ~BUTTON_6_G_PIN;
    }

    if (colorMap[6] == 1) {
      LED_SETTINGS[POV_8_G_REG] &= ~POV_8_G_PIN;
    }
  }
}

void POV_Update(void) {
  if (firstRun) {
    firstRun = 0;
    // runPOV_step(&message_resenv_red[indexTracker+HALF_LENGTH][0],
    // &message_resenv_green[indexTracker][0], indexTracker);
    if (global_visNum == 1) {
      runPOV_step(&message_resenv_red[indexTracker + HALF_LENGTH][0],
                  &message_resenv_green[indexTracker][0], indexTracker);
    } else if (global_visNum == 2) {
      runPOV_step(&lidar_red[indexTracker][0], &lidar_green[indexTracker][0],
                  indexTracker);
    }

    transmitToBuffer();
    indexTracker++;
    return;
  }

  //	hwTriggerCnt++;
  //
  //	if(hwTriggerCnt == (tick_threshold-1)){

  //		hwTriggerCnt = 0;
  if (indexTracker >= (HALF_LENGTH - 1)) {
    HAL_TIM_Base_Stop_IT(&htim3);

    if (global_visNum == 1) {
      runPOV_step(&message_resenv_red[indexTracker + HALF_LENGTH][0],
                  &message_resenv_green[indexTracker][0], indexTracker);
    } else if (global_visNum == 2) {
      runPOV_step(&lidar_red[indexTracker][0], &lidar_green[indexTracker][0],
                  indexTracker);
    }

    transmitToBuffer();

    indexTracker = 0;
    POV_timerActive = 0;

    // switchPOV++;
    firstRun = 1;
  } else {
    if (global_visNum == 1) {
      runPOV_step(&message_resenv_red[indexTracker + HALF_LENGTH][0],
                  &message_resenv_green[indexTracker][0], indexTracker);
    } else if (global_visNum == 2) {
      runPOV_step(&lidar_red[indexTracker][0], &lidar_green[indexTracker][0],
                  indexTracker);
    }

    transmitToBuffer();

    indexTracker++;
  }
}

void runPOV_step(uint8_t* red, uint8_t* green, uint8_t step) {
  memset(LED_SETTINGS, 255, sizeof LED_SETTINGS);

  if (global_visNum == 1) {
    if ((timerPOVstate % 2) == 1) {
      POV_left(green, 0);
      POV_right(red, 1);
    } else {
      POV_left(red, 1);
      POV_right(green, 0);
    }
  } else if (global_visNum == 2) {
    if ((timerPOVstate % 2) == 1) {
      POV_left(green, 0);
      POV_left(red, 1);
      POV_right(green + 15 * MESSAGE_WIDTH, 0);
      POV_right(red + 15 * MESSAGE_WIDTH, 1);
    } else {
      POV_left(green + 15 * MESSAGE_WIDTH, 0);
      POV_left(red + 15 * MESSAGE_WIDTH, 1);
      POV_right(green, 0);
      POV_right(red, 1);
    }
  }
}

// add code to flip POV_L and POV_R during half revolution

void run_message(uint8_t red[][7], uint8_t green[][7],
                 uint32_t cyclePerHalfTurn) {
  // set timer for microsecond resolution

  uint32_t uS_10_needed =
      round(((cyclePerHalfTurn) / ((double)TICK_POV_MICROSEC_10_DIVIDER)) /
            ((double)HALF_LENGTH));

//  if (uS_10_needed == 0) {
//    uS_10_needed = 1;
//  } else if (uS_10_needed >= 65535) {
//    uS_10_needed = 65535;
//  }

  if (POV_timerActive == 0) {
    POV_timerActive = 1;
    // htim3.Init.Period = TIMER_SCALAR * uS_needed;
    // htim3.Instance->PSC = uS_needed * TIMER_SCALAR;

    timerPOVstate = switchPOV;

    // change timer frequency based on rounds-per-second calculation
    htim3.Instance->ARR = (uint16_t)uS_10_needed;

    // run one iteration of the POV display before starting POV timer
    POV_Update();

    // start POV timer
    HAL_TIM_Base_Start_IT(&htim3);
  }
}

void visualizationRun(uint8_t visNum, uint32_t cyclePerHalfTurn) {
  if (visNum == 1) {
    global_visNum = 1;
    run_message(message_resenv_red, message_resenv_green, cyclePerHalfTurn);

  } else if (visNum == 2) {
    lidarPOV_Map = get_lidar_POV_map();
    updatePOV_LidarMatricies(lidarPOV_Map);
    global_visNum = 2;
    run_message(lidar_green, lidar_red, cyclePerHalfTurn);

  } else {
    global_visNum = 1;
    run_message(message_resenv_red, message_resenv_green, cyclePerHalfTurn);
  }
}

void setMatrix(uint8_t matrix[][MESSAGE_WIDTH], uint32_t size, uint16_t length,
               uint8_t reverse) {
  if (reverse == 0) {
    memset(matrix, 1, size);
  } else {
    uint16_t pointerOffset = (MESSAGE_LENGTH - 1) - length;
    memset(&(matrix[pointerOffset][0]), 1, size);
  }
}

uint8_t temp_tracker = 0;
void updatePOV_LidarMatricies(uint32_t lidarPOV_Map) {
  memset(lidar_green, 0,
         sizeof(lidar_green[0][0]) * MESSAGE_LENGTH * MESSAGE_WIDTH);
  memset(lidar_red, 0,
         sizeof(lidar_green[0][0]) * MESSAGE_LENGTH * MESSAGE_WIDTH);

  uint16_t diff;

  if (lidarPOV_Map >= (MESSAGE_LENGTH)) {
    diff = lidarPOV_Map - (MESSAGE_LENGTH);  // can be 0 to 31

    // memset(lidar_green, 1, sizeof(lidar_green[0][0]) * MESSAGE_LENGTH *
    // MESSAGE_WIDTH);

    // setMatrix(lidar_green, sizeof(lidar_green[0][0]) * (diff+1) *
    // MESSAGE_WIDTH, diff, 1);

    setMatrix(lidar_red,
              sizeof(lidar_green[0][0]) *
                  (lidarPOV_Map - (MESSAGE_LENGTH - 1)) * MESSAGE_WIDTH,
              0, 0);
  } else {
    setMatrix(lidar_green,
              sizeof(lidar_green[0][0]) * (lidarPOV_Map + 1) * MESSAGE_WIDTH, 0,
              0);
  }
}

void transmitToBuffer(void) {
  // taskENTER_CRITICAL();
  if (LED_mutex_id != NULL) {
    osMutexWait(LED_mutex_id, osWaitForever);
  }

  HAL_GPIO_WritePin(LED_SS_GPIO_Port, LED_SS_Pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit_IT(&hspi2, LED_SETTINGS, 6);

  // HAL_Delay(1);
  // HAL_GPIO_WritePin(LED_SS_GPIO_Port, LED_SS_Pin, GPIO_PIN_SET);

  if (LED_mutex_id != NULL) {
    osMutexRelease(LED_mutex_id);
  }

  // HAL_GPIO_WritePin(LED_SS_GPIO_Port, LED_SS_Pin, GPIO_PIN_SET);
  // taskEXIT_CRITICAL();
  // HAL_GPIO_WritePin(LED_SS_GPIO_Port, LED_SS_Pin, GPIO_PIN_SET);
}

void cyclePOV_LEDs(uint16_t rate) {
  uint16_t map = 0x0001;
  while (1) {
    map = map << 1;
    if (map == 0) map = 0x0001;
    POV_LEDs(map);
    HAL_Delay(rate);
  }
}

const uint16_t visualization_map1[60] = {
    {MAP_POV_1_R}, {MAP_POV_1_G}, {MAP_POV_1_R}, {MAP_POV_1_G}, {MAP_POV_1_R},
    {MAP_POV_1_G}, {MAP_POV_1_R}, {MAP_POV_1_G}, {MAP_POV_1_R}, {MAP_POV_1_G}};

void POV_LEDs(uint16_t led_map) {
  if ((led_map & MAP_POV_1_R) == MAP_POV_1_R) {
    LED_SETTINGS[POV_1_R_REG] &= ~POV_1_R_PIN;
  } else {
    LED_SETTINGS[POV_1_R_REG] |= POV_1_R_PIN;
  }

  if ((led_map & MAP_POV_2_R) == MAP_POV_2_R) {
    LED_SETTINGS[POV_2_R_REG] &= ~POV_2_R_PIN;
  } else {
    LED_SETTINGS[POV_2_R_REG] |= POV_2_R_PIN;
  }

  if ((led_map & MAP_POV_3_R) == MAP_POV_3_R) {
    LED_SETTINGS[POV_3_R_REG] &= ~POV_3_R_PIN;
  } else {
    LED_SETTINGS[POV_3_R_REG] |= POV_3_R_PIN;
  }

  if ((led_map & MAP_POV_4_R) == MAP_POV_4_R) {
    LED_SETTINGS[POV_4_R_REG] &= ~POV_4_R_PIN;
  } else {
    LED_SETTINGS[POV_4_R_REG] |= POV_4_R_PIN;
  }

  if ((led_map & MAP_POV_5_R) == MAP_POV_5_R) {
    LED_SETTINGS[POV_5_R_REG] &= ~POV_5_R_PIN;
  } else {
    LED_SETTINGS[POV_5_R_REG] |= POV_5_R_PIN;
  }

  if ((led_map & MAP_POV_6_R) == MAP_POV_6_R) {
    LED_SETTINGS[POV_6_R_REG] &= ~POV_6_R_PIN;
  } else {
    LED_SETTINGS[POV_6_R_REG] |= POV_6_R_PIN;
  }

  if ((led_map & MAP_POV_7_R) == MAP_POV_7_R) {
    LED_SETTINGS[POV_7_R_REG] &= ~POV_7_R_PIN;
  } else {
    LED_SETTINGS[POV_7_R_REG] |= POV_7_R_PIN;
  }

  if ((led_map & MAP_POV_8_R) == MAP_POV_8_R) {
    LED_SETTINGS[POV_8_R_REG] &= ~POV_8_R_PIN;
  } else {
    LED_SETTINGS[POV_8_R_REG] |= POV_8_R_PIN;
  }

  if ((led_map & MAP_POV_1_G) == MAP_POV_1_G) {
    LED_SETTINGS[POV_1_G_REG] &= ~POV_1_G_PIN;
  } else {
    LED_SETTINGS[POV_1_G_REG] |= POV_1_G_PIN;
  }

  if ((led_map & MAP_POV_2_G) == MAP_POV_2_G) {
    LED_SETTINGS[POV_2_G_REG] &= ~POV_2_G_PIN;
  } else {
    LED_SETTINGS[POV_2_G_REG] |= POV_2_G_PIN;
  }

  if ((led_map & MAP_POV_3_G) == MAP_POV_3_G) {
    LED_SETTINGS[POV_3_G_REG] &= ~POV_3_G_PIN;
  } else {
    LED_SETTINGS[POV_3_G_REG] |= POV_3_G_PIN;
  }

  if ((led_map & MAP_POV_4_G) == MAP_POV_4_G) {
    LED_SETTINGS[POV_4_G_REG] &= ~POV_4_G_PIN;
  } else {
    LED_SETTINGS[POV_4_G_REG] |= POV_4_G_PIN;
  }

  if ((led_map & MAP_POV_5_G) == MAP_POV_5_G) {
    LED_SETTINGS[POV_5_G_REG] &= ~POV_5_G_PIN;
  } else {
    LED_SETTINGS[POV_5_G_REG] |= POV_5_G_PIN;
  }

  if ((led_map & MAP_POV_6_G) == MAP_POV_6_G) {
    LED_SETTINGS[POV_6_G_REG] &= ~POV_6_G_PIN;
  } else {
    LED_SETTINGS[POV_6_G_REG] |= POV_6_G_PIN;
  }

  if ((led_map & MAP_POV_7_G) == MAP_POV_7_G) {
    LED_SETTINGS[POV_7_G_REG] &= ~POV_7_G_PIN;
  } else {
    LED_SETTINGS[POV_7_G_REG] |= POV_7_G_PIN;
  }

  if ((led_map & MAP_POV_8_G) == MAP_POV_8_G) {
    LED_SETTINGS[POV_8_G_REG] &= ~POV_8_G_PIN;
  } else {
    LED_SETTINGS[POV_8_G_REG] |= POV_8_G_PIN;
  }

  transmitToBuffer();
}

void Set_LEDS(void) {
  LED_SETTINGS[0] = 255;
  LED_SETTINGS[1] = 255;
  LED_SETTINGS[2] = 255;
  LED_SETTINGS[3] = 255;
  LED_SETTINGS[4] = 255;
  LED_SETTINGS[5] = 255;

  transmitToBuffer();
}

void Flush_LEDS(void) {
  LED_SETTINGS[0] = 255;
  LED_SETTINGS[1] = 255;
  LED_SETTINGS[2] = 255;
  LED_SETTINGS[3] = 255;
  LED_SETTINGS[4] = 255;
  LED_SETTINGS[5] = 255;

  transmitToBuffer();
}

void Set_LED(uint8_t reg, uint8_t pin, uint8_t state) {
  // turn on LED
  if (state == 1) {
    LED_SETTINGS[reg] &= ~pin;
  }
  // turn off LED
  else if (state == 0) {
    LED_SETTINGS[reg] |= pin;
  }

  transmitToBuffer();
}

// returns 1 if LED is on, 0 otherwise
// LED is on if the bit in LED_SETTINGS is zero (because its a sink circuit)
uint8_t LED_State(uint8_t reg, uint8_t pin) {
  if ((LED_SETTINGS[reg] & pin) != 0) {
    return 0;  // LED is OFF
  } else {
    return 1;  // LED is ON
  }
}
