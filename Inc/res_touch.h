#ifndef __res_touch_H
#define __res_touch_H

#include "stdint.h"
#include "cmsis_os.h"


#define BUTTON_1_R_PIN		0x10
#define BUTTON_1_R_REG		5
#define BUTTON_1_G_PIN 		0x20
#define BUTTON_1_G_REG		5

#define BUTTON_2_R_PIN		0x01
#define BUTTON_2_R_REG		5
#define BUTTON_2_G_PIN		0x02
#define BUTTON_2_G_REG		5

#define BUTTON_3_R_PIN		0x04
#define BUTTON_3_R_REG		4
#define BUTTON_3_G_PIN		0x08
#define BUTTON_3_G_REG		4

#define BUTTON_4_R_PIN		0x10
#define BUTTON_4_R_REG		1
#define BUTTON_4_G_PIN		0x20
#define BUTTON_4_G_REG		1

#define BUTTON_5_R_PIN		0x40
#define BUTTON_5_R_REG		0
#define BUTTON_5_G_PIN		0x80
#define BUTTON_5_G_REG		0

#define BUTTON_6_R_PIN		0x04
#define BUTTON_6_R_REG		0
#define BUTTON_6_G_PIN		0x08
#define BUTTON_6_G_REG		0

#define BUTTON_7_R_PIN		0x40
#define BUTTON_7_R_REG		3
#define BUTTON_7_G_PIN		0x80
#define BUTTON_7_G_REG		3

#define BUTTON_8_R_PIN		0x40
#define BUTTON_8_R_REG		4
#define BUTTON_8_G_PIN		0x80
#define BUTTON_8_G_REG		4

#define BUTTON_9_R_PIN		0x01
#define BUTTON_9_R_REG		1
#define BUTTON_9_G_PIN		0x02
#define BUTTON_9_G_REG		1

#define BUTTON_10_R_PIN		0x01
#define BUTTON_10_R_REG		2
#define BUTTON_10_G_PIN		0x02
#define BUTTON_10_G_REG		2

osTimerId povExitTimerHandle;

void ResistiveTouchSampler(void);
void buttonStateMachine(void);
void disable_buttons(void);
void enable_buttons(void);
uint8_t isButtonEnabled(void);
uint8_t getBitCrush(void);


#endif
