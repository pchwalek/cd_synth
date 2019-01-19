#include "lidar.h"
#include "I2C.h"
#include "vl53l1_api.h"
#include "led_cntrl.h"
#include "wave_synth.h"
#include "res_touch.h"

#include "usart.h"

#define MEASUREMENT_MULTIPLIER	10
#define MAX_LIDAR_MEASUREMENT   1300 //2m
#define MEASUREMENT_DIVISOR		20

#define MEASUREMENT_POV_DIVISOR 10

#define ALPHA_MEAS				0.9
#define BETA_MEAS				0.1
#define THRESH_MEAS				3

#define SYSTEM__INTERRUPT_CLEAR 	0x0086
#define SYSTEM__MODE_START          0x0087
#define VL53L1_ADDR					0x52

#define RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI   0x0096 //AA
#define RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO   0x0097 //AB
#define RESULT__RANGE_STATUS        				   		0x0089

uint16_t lidar_measurement;
int16_t measurement_hist = -1;

VL53L1_Dev_t 	dev;
VL53L1_DEV	Dev 		= 	&dev;
uint16_t 	lidar_map	=	0;
uint32_t 	POV_map		=	0;

volatile uint8_t LED_Lidar_Active = 0;

int16_t oldLidarMeasurement = 0;

void activateLidarLED(void){
	LED_Lidar_Active = 1;
}

void deactiveLidarLED(void){
	LED_Lidar_Active = 0;
}

//turn on LIDAR (only works if 3.3V Power Switch is set (enabled) )
void activateLidar(void){
    HAL_GPIO_WritePin(VL_XSHUT_GPIO_Port, VL_XSHUT_Pin, GPIO_PIN_SET);
}

//turn off LIDAR
void deactivateLidar(void){
    HAL_GPIO_WritePin(VL_XSHUT_GPIO_Port, VL_XSHUT_Pin, GPIO_PIN_RESET);
}

// gen LED map for when disk is not spinning
void genMap(int16_t* measurement){
	lidar_map = 0x0001 << ( ((uint16_t) (*measurement / ((float) MEASUREMENT_DIVISOR)) ) % 16);
}

// gen LED activation value for when disk is spinning
void genPOV_Map(int16_t* measurement){
	POV_map = ( ((uint16_t) (*measurement / ((float) MEASUREMENT_POV_DIVISOR)) ) % 64);
}

uint32_t get_lidar_POV_map(void){
	return POV_map;
}

void LidarMeasurementHandler(volatile int16_t* measurement){
//#ifdef DEBUG_PRINT
//  HAL_UART_Transmit(&huart3, "Lidar handler\n\r", sizeof("Lidar handler\n\r"), 100);
//#endif
	if(*measurement > MAX_LIDAR_MEASUREMENT){
	    //ensure capacitive sensing didnt toggle on at this moment
	    if(isCapModeActive() == 0){
		    turnSoundOff();
	    }
		*measurement = MAX_LIDAR_MEASUREMENT;
	}
	else{
	    turnSoundOn();
	}

	// flip the measurements so high frequencies are when you get close to the disk
	//*measurement = MAX_LIDAR_MEASUREMENT - *measurement;

//#ifdef DEBUG_PRINT
//  HAL_UART_Transmit(&huart3, "Lidar freq\n\r", sizeof("Lidar freq\n\r"), 100);
//#endif
	// calculate frequency of wavetable based on measurement
	calcLidarFreq(measurement);

//#ifdef DEBUG_PRINT
//  HAL_UART_Transmit(&huart3, "Lidar button enabled\n\r", sizeof("Lidar button eneabled\n\r"), 100);
//#endif
	// non-spinning LED display mode
	if(isButtonEnabled() == 1){
		genMap(measurement);

//#ifdef DEBUG_PRINT
//  HAL_UART_Transmit(&huart3, "POV LED\n\r", sizeof("POV LED\n\r"), 100);
//#endif
		POV_LEDs(lidar_map);
	}
//
//#ifdef DEBUG_PRINT
//  HAL_UART_Transmit(&huart3, "Lidar gen pov map\n\r", sizeof("Lidar gen pov map\n\r"), 100);
//#endif
	genPOV_Map(measurement);


}

void LidarMeasurement(void)
{
  int status;
  //int IntCount;

  activateLidar();
  HAL_Delay(500);
  osDelay(5);
  Dev->I2cHandle = &hi2c3;
  Dev->I2cDevAddr = 0x52;
  static VL53L1_RangingMeasurementData_t RangingData;
  //printf("Autonomous Ranging Test\n");
  status = VL53L1_WaitDeviceBooted(Dev);
  status = VL53L1_DataInit(Dev);
  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 20000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 22);
  status = VL53L1_StartMeasurement(Dev);
  status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
  while(1){
	  	osSemaphoreWait( lidarSampleReadySemaphoreHandle, osWaitForever);
#ifdef DEBUG_PRINT
  HAL_UART_Transmit(&huart3, "start_lidar\n\r", sizeof("start_lidar\n\r"), 100);
#endif
		status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
		//VL53L1_DEVICERESULTSLEVEL_FULL
		//LidarMeasurementHandler(&temp_meas);
		    taskENTER_CRITICAL();

//#ifdef DEBUG_PRINT
//
//  HAL_UART_Transmit(&huart3, "lidar_range\n\r", sizeof("lidar_range\n\r"), 100);
//#endif
		if(status == VL53L1_RANGESTATUS_RANGE_VALID){
//#ifdef DEBUG_PRINT
//  char totalTimeString[10];
//  itoa(measurement_hist, totalTimeString, 10);
//  HAL_UART_Transmit(&huart3, totalTimeString, sizeof(totalTimeString), 100);
//
//  HAL_UART_Transmit(&huart3, "\n\rrange_valid\n\r", sizeof("\n\rrange_valid\n\r"), 100);
//
//  char totalTimeString2[10];
//  itoa(RangingData.RangeMilliMeter, totalTimeString2, 10);
//  HAL_UART_Transmit(&huart3, totalTimeString2, sizeof(totalTimeString2), 100);
//
//
//#endif
			//first measurement
			if(measurement_hist == -1){
//#ifdef DEBUG_PRINT
//  HAL_UART_Transmit(&huart3, "\n\r 1st statement\n\r", sizeof("\n\r 1st statement\n\r"), 100);
//#endif
				measurement_hist = RangingData.RangeMilliMeter;
				//LidarMeasurementHandler(&RangingData.RangeMilliMeter);
			}
			else if(abs(measurement_hist-RangingData.RangeMilliMeter) > THRESH_MEAS){
//#ifdef DEBUG_PRINT
//  HAL_UART_Transmit(&huart3, " 2nd statement\n\r", sizeof(" 2nd statement\n\r"), 100);
//#endif
				measurement_hist = RangingData.RangeMilliMeter;
				//LidarMeasurementHandler(&RangingData.RangeMilliMeter);
			}
			else{
//#ifdef DEBUG_PRINT
//  HAL_UART_Transmit(&huart3, " 3rd statement\n\r", sizeof(" 3rd statement\n\r"), 100);
//#endif
				measurement_hist = measurement_hist * ALPHA_MEAS + RangingData.RangeMilliMeter * BETA_MEAS;
			}

			if( oldLidarMeasurement != measurement_hist){
			    oldLidarMeasurement = measurement_hist;
			    LidarMeasurementHandler(&oldLidarMeasurement);
			}


		}
		    taskEXIT_CRITICAL();

//#ifdef DEBUG_PRINT
//  HAL_UART_Transmit(&huart3, "lidar_clear\n\r", sizeof("lidar_clear\n\r"), 100);
//#endif
		status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
		//VL53L1_clear_int_and_enable_range();
		if(isLidarModeActive() == 0) lidarStop();
#ifdef DEBUG_PRINT
  HAL_UART_Transmit(&huart3, "stop_lidar\n\r", sizeof("stop_lidar\n\r"), 100);
#endif
	}

}

volatile int status;
volatile int IntCount;
//static VL53L1_RangingMeasurementData_t RangingData;

void VL53L1_clear_int_and_enable_range(void){
	uint8_t packet;

	packet = 0x01;
	HAL_I2C_Mem_Write(&hi2c3, VL53L1_ADDR<<1, SYSTEM__INTERRUPT_CLEAR, 1, &packet, 1, 1);

	packet = 0x40;
	HAL_I2C_Mem_Write(&hi2c3, VL53L1_ADDR<<1, SYSTEM__MODE_START, 1, &packet, 1, 1);
}

void VL53L1_read_meas(void){
	uint8_t packet[17] = {0};

	//HAL_I2C_Mem_Write(&hi2c3, VL53L1_ADDR<<1, SYSTEM__MODE_START, 1, &packet, 1, 1);

	packet[0] = (RESULT__RANGE_STATUS >> 8) & 0xFF;
	packet[1] = RESULT__RANGE_STATUS       & 0xFF;
	HAL_I2C_Master_Transmit(&hi2c3, VL53L1_ADDR<<1, packet, 2, 1);
	HAL_I2C_Mem_Read(&hi2c3, VL53L1_ADDR<<1, RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI, 1, packet, 17, 1);

	lidar_measurement  = ((uint16_t)packet[13]) << 8; // high byte
	lidar_measurement |= packet[14];      // low byte
}

void lidarStop(void){
	status = VL53L1_StopMeasurement(Dev);
}

void lidarStart(void){
	status = VL53L1_StartMeasurement(Dev);
	VL53L1_ClearInterruptAndStartMeasurement(Dev);
}

