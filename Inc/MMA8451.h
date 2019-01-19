/**************************************************************************/
/*!
    @file     MMA8451.h
    @author   K. Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit MMA8451 Accel breakout board
    ----> https://www.adafruit.com/products/2019

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/
#ifndef _MMA8451_H_
#define _MMA8451_H_
#include "Adafruit_Sensor.h"
#include "stdint.h"
#include <stdbool.h>
#include <math.h>
#include "cmsis_os.h"


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define MMA8451_DEFAULT_ADDRESS                 (0x1C)    // if A is GND, its 0x1C
/*=========================================================================*/

#define MMA8451_REG_OUT_X_MSB     0x01
#define MMA8451_REG_SYSMOD        0x0B
#define MMA8451_REG_WHOAMI        0x0D
#define MMA8451_REG_XYZ_DATA_CFG  0x0E
#define MMA8451_REG_PL_STATUS     0x10
#define MMA8451_REG_PL_CFG        0x11
#define MMA8451_REG_CTRL_REG1     0x2A
#define MMA8451_REG_CTRL_REG2     0x2B
#define MMA8451_REG_CTRL_REG4     0x2D
#define MMA8451_REG_CTRL_REG5     0x2E



#define MMA8451_PL_PUF            0
#define MMA8451_PL_PUB            1
#define MMA8451_PL_PDF            2
#define MMA8451_PL_PDB            3
#define MMA8451_PL_LRF            4
#define MMA8451_PL_LRB            5
#define MMA8451_PL_LLF            6
#define MMA8451_PL_LLB            7

osSemaphoreId accSampleSemaphoreHandle;
osTimerId accSampleTimerHandle;

typedef enum
{
  MMA8451_RANGE_8_G           = 0b10,   // +/- 8g
  MMA8451_RANGE_4_G           = 0b01,   // +/- 4g
  MMA8451_RANGE_2_G           = 0b00    // +/- 2g (default value)
} mma8451_range_t;


/* Used with register 0x2A (MMA8451_REG_CTRL_REG1) to set bandwidth */
typedef enum
{
  MMA8451_DATARATE_800_HZ     = 0b000, //  800Hz
  MMA8451_DATARATE_400_HZ     = 0b001, //  400Hz
  MMA8451_DATARATE_200_HZ     = 0b010, //  200Hz
  MMA8451_DATARATE_100_HZ     = 0b011, //  100Hz
  MMA8451_DATARATE_50_HZ      = 0b100, //   50Hz
  MMA8451_DATARATE_12_5_HZ    = 0b101, // 12.5Hz
  MMA8451_DATARATE_6_25HZ     = 0b110, // 6.25Hz
  MMA8451_DATARATE_1_56_HZ    = 0b111, // 1.56Hz

  MMA8451_DATARATE_MASK       = 0b111
} mma8451_dataRate_t;

void calcSpin(void);
bool MMA8451_begin(void);
mma8451_dataRate_t MMA8451_getDataRate(void);
void MMA8451_setRange(mma8451_range_t range);
void MMA8451_setDataRate(mma8451_dataRate_t dataRate);
bool MMA8451_getEvent(sensors_event_t *event);
void MMA8451_getSensor(sensor_t *sensor);
uint8_t MMA8451_getOrientation(void);
void MMA8451_writeRegister8(uint8_t reg, uint8_t value);
uint8_t MMA8451_readRegister8(uint8_t reg);
mma8451_range_t MMA8451_getRange(void);
void calcAngularVelocity(void);
void accGiveSemaphore(void);
void MMA8451_read(uint8_t* sample);
void accelerometerThread(void);
float calculateAngle(float x_g, float y_g, float z_g);

volatile int16_t x, y, z;
volatile float x_g, y_g, z_g;
int32_t _sensorID;
int8_t  _i2caddr;

osMutexId  (I2C3_mutex_id); // Mutex ID

#endif
