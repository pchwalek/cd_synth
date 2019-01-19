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

#include <MMA8451.h>
#include "I2C.h"
#include <stdio.h>
#include <string.h>
#include "led_cntrl.h"
#include "stdlib.h"
#include "wave_synth.h"
#include "filter.h"

#include "usart.h"


#define ACC_DIST 	0.056769762 //in meters
//#define HZ_1		6.28318
#define HZ_1		0.4
#define PI_DIV_2	1.5707963268


uint16_t 		POV_map2	=	0;
volatile float angVel = -1;
volatile uint8_t sample[6];
const float gravityVector[] = {0,0,-1};
volatile float dotProd;
volatile float angle;

uint8_t samplePacket;

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
//static inline uint8_t i2cread(void) {
//  #if ARDUINO >= 100
//  return Wire.read();
//  #else
//  return Wire.receive();
//  #endif
//}
//
//static inline void i2cwrite(uint8_t x) {
//
//	HAL_I2C_Mem_Write(&hi2c2, CAP1214_ADDR<<1, CNFG_REG_4, 1, &packet, 1, 1);
//
//	#if ARDUINO >= 100
//  Wire.write((uint8_t)x);
//  #else
//  Wire.send(x);
//  #endif
//}

void accGiveSemaphore(void){
	osSemaphoreRelease (accSampleSemaphoreHandle);
}


void accelerometerThread(void){
	  MMA8451_begin();
	  MMA8451_setRange(MMA8451_RANGE_4_G);

	while(1){
	  osSemaphoreWait (accSampleSemaphoreHandle, osWaitForever);
#ifdef DEBUG_PRINT
  HAL_UART_Transmit(&huart3, "start_acc\n\r", sizeof("start_acc\n\r"), 100);
#endif
  //taskENTER_CRITICAL();
	  HAL_I2C_Mem_Read_IT(&hi2c2, _i2caddr<<1, MMA8451_REG_OUT_X_MSB, 1, sample, 6);
	  //taskEXIT_CRITICAL();
	  osDelay(10);
//	  osMutexWait(I2C3_mutex_id, osWaitForever);

	  MMA8451_read(sample);

//	  sensors_event_t event;
//	  MMA8451_getEvent(&event);
	  //if (I2C3_mutex_id != NULL) osMutexWait(I2C3_mutex_id, osWaitForever);
	  //calcFilterFreqAcc(x_g, y_g, z_g);

	  angle = calculateAngle(x_g, y_g, z_g);
	  setCutoffFreq(angle);

//	  /* Get the orientation of the sensor */
//	  uint8_t o = MMA8451_getOrientation();
//
//	  switch (o) {
//	      case MMA8451_PL_PUF:
//	    	  setFilter(0);
////	    	  strncpy(str_sw, "Portrait Up Front", 50);
////	    	HAL_UART_Transmit(&huart3, (uint8_t*) str_sw, sizeof(str_sw), 50);
//	        break;
//	      case MMA8451_PL_PUB:
//	    	  setFilter(1);
////	    	  strncpy(str_sw, "Portrait Up Back", 50);
////	    	HAL_UART_Transmit(&huart3, (uint8_t*) str_sw, sizeof(str_sw), 50);
//	        break;
//	      case MMA8451_PL_PDF:
//	    	  setFilter(2);
////	    	  strncpy(str_sw, "Portrait Down Front", 50);
////	    	HAL_UART_Transmit(&huart3, (uint8_t*) str_sw, sizeof(str_sw), 50);
//	        break;
//	      case MMA8451_PL_PDB:
//	    	  setFilter(3);
////	    	  strncpy(str_sw, "Portrait Down Back", 50);
////	    	HAL_UART_Transmit(&huart3, (uint8_t*) str_sw, sizeof(str_sw), 50);
//	        break;
//	      case MMA8451_PL_LRF:
//	    	  setFilter(4);
////	    	  strncpy(str_sw, "Landscape Right Front", 50);
////	    	  HAL_UART_Transmit(&huart3, (uint8_t*) str_sw, sizeof(str_sw), 50);
//	        break;
//	      case MMA8451_PL_LRB:
//	    	  setFilter(5);
////	    	  strncpy(str_sw, "Landscape Right Back", 50);
////	    	  HAL_UART_Transmit(&huart3, (uint8_t*) str_sw, sizeof(str_sw), 50);
//	        break;
//	      case MMA8451_PL_LLF:
//	    	  setFilter(6);
////	    	  strncpy(str_sw, "Landscape Left Front", 50);
////	    	  HAL_UART_Transmit(&huart3, (uint8_t*) str_sw, sizeof(str_sw), 50);
//	        break;
//	      case MMA8451_PL_LLB:
//	    	  setFilter(7);
////	    	  strncpy(str_sw, "Landscape Left Back", 50);
////	    	  HAL_UART_Transmit(&huart3, (uint8_t*) str_sw, sizeof(str_sw), 50);
//	        break;
//	      }
	  //calcSpin();
#ifdef DEBUG_PRINT
  HAL_UART_Transmit(&huart3, "stop_acc\n\r", sizeof("stop_acc\n\r"), 100);
#endif
	}
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void MMA8451_writeRegister8(uint8_t reg, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c2, _i2caddr<<1, reg, 1, &value, 1, 1);
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t MMA8451_readRegister8(uint8_t reg) {
    
//undocumented version of requestFrom handles repeated starts on Arduino Due
//#ifdef __SAM3X8E__
//    Wire.requestFrom(_i2caddr, 1, reg, 1, true);
//#else
//    //I don't know - maybe the other verion of requestFrom works on all platforms.
//    //  honestly, I don't want to go through and test them all.  Doing it this way
//    //  is already known to work on everything else
//    Wire.beginTransmission(_i2caddr);
//    i2cwrite(reg);
//    Wire.endTransmission(false); // MMA8451 + friends uses repeated start!!
//    Wire.requestFrom(_i2caddr, 1);
//#endif

    HAL_I2C_Mem_Read(&hi2c2, _i2caddr<<1, reg, 1, &sample, 1, 2);
    
    return samplePacket;
}

/**************************************************************************/
/*!
    @brief  Instantiates a new MMA8451 class in I2C mode
*/
/**************************************************************************/
void MMA8451_ID(int32_t sensorID) {
  _sensorID = sensorID;
}


/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool MMA8451_begin(void) {
  _i2caddr = MMA8451_DEFAULT_ADDRESS;

  /* Check connection */
  uint8_t deviceid = MMA8451_readRegister8(MMA8451_REG_WHOAMI);
  //HAL_Delay(4000);

  if (deviceid != 0x1A)
  {
    /* No MMA8451 detected ... return false */
    //Serial.println(deviceid, HEX);
    return false;
  }

  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG2, 0x40); // reset

  while (MMA8451_readRegister8(MMA8451_REG_CTRL_REG2) & 0x40);

  // enable 4G range & High Pass Thresh ( | 0x10)
  MMA8451_writeRegister8(MMA8451_REG_XYZ_DATA_CFG, MMA8451_RANGE_4_G); // | 0x10

  // oversampling -> high
  MMA8451_writeRegister8(0x0F, 0x03);

  // High res
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG2, 0x02);
  // DRDY on INT1
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG4, 0x01);
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG5, 0x01);

  // Turn on orientation config
  MMA8451_writeRegister8(MMA8451_REG_PL_CFG, 0x40);

  // Activate at max rate, low noise mode
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, 0x01 | 0x04);

  /*
  for (uint8_t i=0; i<0x30; i++) {
    Serial.print("$");
    Serial.print(i, HEX); Serial.print(" = 0x");
    Serial.println(MMA8451_readRegister8(i), HEX);
  }
  */

  return true;
}

void MMA8451_read(uint8_t* sample) {


  x = sample[0]; x <<= 8; x |= sample[1]; x >>= 2;
  y = sample[2]; y <<= 8; y |= sample[3]; y >>= 2;
  z = sample[4]; z <<= 8; z |= sample[5]; z >>= 2;


////  uint8_t range = MMA8451_getRange();
//  uint16_t divider = 1;
////  if (range == MMA8451_RANGE_8_G) divider = 1024;
////  if (range == MMA8451_RANGE_4_G) divider = 2048;
//  if (range == MMA8451_RANGE_2_G) divider = 4096;

  x_g = (float)x / 2048;
  y_g = (float)y / 2048;
  z_g = (float)z / 2048;

}

void ledOut(int8_t shift){
	POV_map2 = 0x0001 << (( shift) % 15);
	POV_LEDs(POV_map2);
}

volatile int8_t number = 0;
void calcSpin(void){
	calcAngularVelocity();
	if(angVel < HZ_1){
		//ledOut(1);
	}
	else if(angVel < (2*HZ_1)){
		//ledOut(2);
	}
	else if(angVel < (3*HZ_1)){
//		ledOut(3);
		}
	else if(angVel < (4*HZ_1)){
//		ledOut(4);
		}
	else if(angVel < (5*HZ_1)){
//		ledOut(5);
		}
	else if(angVel < (6*HZ_1)){
//		ledOut(6);
		}
	else if(angVel < (7*HZ_1)){
//		ledOut(7);
		}
	else if(angVel < (8*HZ_1)){
//		ledOut(8);
		}
	else if(angVel < (9*HZ_1)){
//		ledOut(9);
		}
	else if(angVel < (10*HZ_1)){
//		ledOut(10);
		}
	else if(angVel < (11*HZ_1)){
//		ledOut(11);
		}
	else if(angVel < (12*HZ_1)){
//		ledOut(12);
			}
	else if(angVel < (13*HZ_1)){
//		ledOut(13);
			}
	else if(angVel < (14*HZ_1)){
//		ledOut(14);
			}
	else{
//		ledOut(15);
	}
}

//input: x_acc is in m/s^2
//output: angular velocity in
volatile float temp;
void calcAngularVelocity(void){
	temp = fabs(x_g)/((float) ACC_DIST);
	angVel = sqrt(fabs(x_g)/((float) ACC_DIST));
}

/**************************************************************************/
/*!
    @brief  Read the orientation:
    Portrait/Landscape + Up/Down/Left/Right + Front/Back
*/
/**************************************************************************/
uint8_t MMA8451_getOrientation(void) {
  return MMA8451_readRegister8(MMA8451_REG_PL_STATUS) & 0x07;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void MMA8451_setRange(mma8451_range_t range)
{
  uint8_t reg1 = MMA8451_readRegister8(MMA8451_REG_CTRL_REG1);
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, 0x00);            // deactivate
  MMA8451_writeRegister8(MMA8451_REG_XYZ_DATA_CFG, (range & 0x3));  // | 0x10
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, reg1 | 0x01);     // activate
}

/**************************************************************************/
/*!
    @brief  Gets the g range for the accelerometer
*/
/**************************************************************************/
mma8451_range_t MMA8451_getRange(void)
{
  /* Read the data format register to preserve bits */
  return (mma8451_range_t)(MMA8451_readRegister8(MMA8451_REG_XYZ_DATA_CFG) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the MMA8451 (controls power consumption)
*/
/**************************************************************************/
void MMA8451_setDataRate(mma8451_dataRate_t dataRate)
{
  uint8_t ctl1 = MMA8451_readRegister8(MMA8451_REG_CTRL_REG1);
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, 0x00);            // deactivate
  ctl1 &= ~(MMA8451_DATARATE_MASK << 3);                  // mask off bits
  ctl1 |= (dataRate << 3);
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, ctl1 | 0x01);     // activate
}

/**************************************************************************/
/*!
    @brief  Gets the data rate for the MMA8451 (controls power consumption)
*/
/**************************************************************************/
mma8451_dataRate_t MMA8451_getDataRate(void)
{
  return (mma8451_dataRate_t)((MMA8451_readRegister8(MMA8451_REG_CTRL_REG1) >> 3) & MMA8451_DATARATE_MASK);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool MMA8451_getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;

  HAL_I2C_Mem_Read_IT(&hi2c2, _i2caddr<<1, MMA8451_REG_OUT_X_MSB, 1, sample, 6);

  MMA8451_read(sample);

  // Convert Acceleration Data to m/s^2
  event->acceleration.x = x_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = y_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = z_g * SENSORS_GRAVITY_STANDARD;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void MMA8451_getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "MMA8451", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 0;
  sensor->min_value   = 0;
  sensor->resolution  = 0;
}


float calculateAngle(float x_g, float y_g, float z_g){
	float accVector[] = {x_g, y_g, z_g};

	arm_dot_prod_f32(accVector,gravityVector,3,&dotProd);

	return (float) acos(dotProd/ ( sqrt(x_g*x_g + y_g*y_g + z_g*z_g)));
//	return angle;
}
