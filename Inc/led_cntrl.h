#ifndef __led_cntrl_H
#define __led_cntrl_H

#include "stdint.h"
#include "cmsis_os.h"


#define CAP_1_LED_PIN		0x20
#define CAP_1_LED_PORT		3

#define CAP_2_LED_PIN		0x10
#define CAP_2_LED_PORT		3

#define CAP_3_LED_PIN		0x08
#define CAP_3_LED_PORT		3

#define CAP_4_LED_PIN		0x04
#define CAP_4_LED_PORT		3

#define CAP_5_LED_PIN		0x02
#define CAP_5_LED_PORT		3

#define CAP_6_LED_PIN		0x01
#define CAP_6_LED_PORT		3

#define CAP_7_LED_PIN		0x80
#define CAP_7_LED_PORT		2

#define CAP_8_LED_PIN		0x40
#define CAP_8_LED_PORT		2

#define CAP_9_LED_PIN		0x20
#define CAP_9_LED_PORT		2

#define CAP_10_LED_PIN		0x10
#define CAP_10_LED_PORT		2

#define CAP_11_LED_PIN		0x08
#define CAP_11_LED_PORT		2

#define CAP_12_LED_PIN		0x04
#define CAP_12_LED_PORT		2

#define POV_1_R_PIN			0x40
#define POV_1_R_REG			5
#define POV_1_G_PIN 		0x80
#define POV_1_G_REG			5

#define POV_2_R_PIN			0x04
#define POV_2_R_REG			5
#define POV_2_G_PIN 		0x08
#define POV_2_G_REG			5

#define POV_3_R_PIN			0x10
#define POV_3_R_REG			4
#define POV_3_G_PIN 		0x20
#define POV_3_G_REG			4

#define POV_4_R_PIN			0x01
#define POV_4_R_REG			4
#define POV_4_G_PIN 		0x02
#define POV_4_G_REG			4

#define POV_5_R_PIN			0x40
#define POV_5_R_REG			1
#define POV_5_G_PIN 		0x80
#define POV_5_G_REG			1

#define POV_6_R_PIN			0x04
#define POV_6_R_REG			1
#define POV_6_G_PIN 		0x08
#define POV_6_G_REG			1

#define POV_7_R_PIN			0x10
#define POV_7_R_REG			0
#define POV_7_G_PIN 		0x20
#define POV_7_G_REG			0

#define POV_8_R_PIN			0x01
#define POV_8_R_REG			0
#define POV_8_G_PIN 		0x02
#define POV_8_G_REG			0

#define MAP_POV_1_R			0x0001
#define MAP_POV_2_R			0x0002
#define MAP_POV_3_R			0x0004
#define MAP_POV_4_R			0x0008
#define MAP_POV_5_R			0x0010
#define MAP_POV_6_R			0x0020
#define MAP_POV_7_R			0x0040
#define MAP_POV_8_R			0x0080

#define MAP_POV_1_G			0x0100
#define MAP_POV_2_G			0x0200
#define MAP_POV_3_G			0x0400
#define MAP_POV_4_G			0x0800
#define MAP_POV_5_G			0x1000
#define MAP_POV_6_G			0x2000
#define MAP_POV_7_G			0x4000
#define MAP_POV_8_G			0x8000

#define MESSAGE_LENGTH	32
#define MESSAGE_WIDTH	7
#define HALF_LENGTH		16

osMutexId  (LED_mutex_id); // Mutex ID

volatile uint8_t LED_SETTINGS[6];

void runPOV_step(uint8_t* red, uint8_t* green, uint8_t step);
void POV_Update(void);
void POV_handler(uint64_t RPR);
void transmitToBuffer(void);
void usTick(void);
void run_message(uint8_t red[][7], uint8_t green[][7], uint32_t  cyclePerHalfTurn);
void POV_right(uint8_t* colorMap, uint8_t color);
void POV_left(uint8_t* colorMap, uint8_t color);
void cyclePOV_LEDs(uint16_t rate);
void POV_LEDs(uint16_t led_map);
void Set_LEDS(void);
void Flush_LEDS(void);
void Set_LED(uint8_t reg, uint8_t pin, uint8_t state);
uint8_t LED_State(uint8_t reg, uint8_t pin);
void visualizationRun(uint8_t visNum, uint32_t cyclePerHalfTurn);
void updatePOV_LidarMatricies(uint32_t lidarPOV_Map);
void setMatrix(uint8_t matrix[][MESSAGE_WIDTH], uint32_t size, uint16_t length, uint8_t reverse);

#endif
