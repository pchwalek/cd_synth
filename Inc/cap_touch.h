#ifndef __cap_touch_H
#define __cap_touch_H

#include "cmsis_os.h"


#define LEFT_BUTTON_PIN		0x01
#define LEFT_BUTTON_PORT	0

#define RIGHT_BUTTON_PIN	0x80
#define RIGHT_BUTTON_PORT	1

#define KEY_1_PIN			0x02
#define KEY_1_PORT			0

#define KEY_2_PIN			0x04
#define KEY_2_PORT			0

#define KEY_3_PIN			0x08
#define KEY_3_PORT			0

#define KEY_4_PIN			0x10
#define KEY_4_PORT			0

#define KEY_5_PIN			0x20
#define KEY_5_PORT			0

#define KEY_6_PIN			0x01
#define KEY_6_PORT			1

#define KEY_7_PIN			0x02
#define KEY_7_PORT			1

#define KEY_8_PIN			0x04
#define KEY_8_PORT			1

#define KEY_9_PIN			0x08
#define KEY_9_PORT			1

#define KEY_10_PIN			0x10
#define KEY_10_PORT			1

#define KEY_11_PIN			0x20
#define KEY_11_PORT			1

#define KEY_12_PIN			0x40
#define KEY_12_PORT			1

#define CAP1214_ADDR		0x28

#define SENSOR_1_DELTA_CNT	0x10
#define CNFG_REG_4			0x40
#define MULT_TOUCH_REG		0x2A
#define CALIBRATION_REG		0x25
#define DATA_SENS_REG		0x1F
#define RECAL_REG			0x2F
#define MAIN_STATUS			0x00

osSemaphoreId capSampleSemaphoreHandle;
osTimerId capSampleTimerHandle;

void capGiveSemaphore(void);
void Setup_Cap_Touch(void);
void Read_Cap_Touch(void);
void Sample_Cap_Touch(void);
void Reset_Cap_INT(void);

#endif
