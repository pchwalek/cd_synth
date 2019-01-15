#include "lidar.h"
#include "I2C.h"
#include "vl53l1_api.h"
#include "led_cntrl.h"
#include "wave_synth.h"
#include "res_touch.h"

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
VL53L1_DEV		Dev 	= 	&dev;
uint16_t 		lidar_map	=	0;
uint32_t 		POV_map	=	0;

volatile uint8_t LED_Lidar_Active = 0;

void activateLidarLED(void){
	LED_Lidar_Active = 1;
}

void deactiveLidarLED(void){
	LED_Lidar_Active = 0;
}

void activateLidar(void){
    //turn on LIDAR (only works if 3.3V Power Switch is set)
    HAL_GPIO_WritePin(VL_XSHUT_GPIO_Port, VL_XSHUT_Pin, GPIO_PIN_SET);
}

void deactivateLidar(void){
    //turn off LIDAR
    HAL_GPIO_WritePin(VL_XSHUT_GPIO_Port, VL_XSHUT_Pin, GPIO_PIN_RESET);
}

void genMap(int16_t* measurement){
	lidar_map = 0x0001 << ( ((uint16_t) (*measurement / ((float) MEASUREMENT_DIVISOR)) ) % 15);
}

void genPOV_Map(int16_t* measurement){
	POV_map = ( ((uint16_t) (*measurement / ((float) MEASUREMENT_POV_DIVISOR)) ) % 64);
}

uint32_t get_lidar_POV_map(void){
	return POV_map;
}

void LidarMeasurementHandler(volatile int16_t* measurement){
	if(*measurement > MAX_LIDAR_MEASUREMENT){
		*measurement = MAX_LIDAR_MEASUREMENT;
	}

	calcLidarFreq(measurement);

	if(isButtonEnabled() == 1){
		genMap(measurement);
		POV_LEDs(lidar_map);
	}

	genPOV_Map(measurement);


}

void LidarMeasurement(void)
{
	volatile int status;
	volatile int IntCount;

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
//	osSemaphoreWait( lidarSampleReadySemaphoreHandle, osWaitForever);
//	taskENTER_CRITICAL();
//	status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
//	taskEXIT_CRITICAL();
////	if(status==0){
//////					printf("%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
//////									RangingData.SignalRateRtnMegaCps/65536.0,RangingData.AmbientRateRtnMegaCps/65336.0);
////	}
//	LidarMeasurementHandler(&RangingData.RangeMilliMeter);
//	taskENTER_CRITICAL();
//	status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
//	taskEXIT_CRITICAL();

	//VL53L1_read_meas();
	  	osSemaphoreWait( lidarSampleReadySemaphoreHandle, osWaitForever);
		status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
		//VL53L1_DEVICERESULTSLEVEL_FULL
		//LidarMeasurementHandler(&temp_meas);
		taskENTER_CRITICAL();
		if(status == VL53L1_RANGESTATUS_RANGE_VALID){
			//first measruement
			if(measurement_hist == -1){
				measurement_hist = RangingData.RangeMilliMeter;
				LidarMeasurementHandler(&RangingData.RangeMilliMeter);
			}
			else if(abs(measurement_hist-RangingData.RangeMilliMeter) > THRESH_MEAS){
				measurement_hist = RangingData.RangeMilliMeter;
				LidarMeasurementHandler(&RangingData.RangeMilliMeter);
			}
			else{
				measurement_hist = measurement_hist * ALPHA_MEAS + RangingData.RangeMilliMeter * BETA_MEAS;
			}

		}
		taskEXIT_CRITICAL();
		status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
		//VL53L1_clear_int_and_enable_range();
		if(isLidarModeActive() == 0) lidarStop();
	}
//
//	if(status){
//		//printf("VL53L1_StartMeasurement failed \n");
//		while(1);
//	}
//	if (isInterrupt){
//		do // interrupt mode
//		{
//		 __WFI();
//		 if(IntCount !=0 ){
//				IntCount=0;
//				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
//				if(status==0){
////					printf("%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
////									RangingData.SignalRateRtnMegaCps/65536.0,RangingData.AmbientRateRtnMegaCps/65336.0);
//				}
//				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
//			}
//		}
//		while(1);
//	}
//	else{
//		do // polling mode
//		{
//		  status = VL53L1_WaitMeasurementDataReady(Dev);
//			if(!status)
//			{
//				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
//				if(status==0){
////					printf("%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
////									(RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
//				}
//				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
//			}
//	isLidarModeActive	}
//		while (1);

//  return status;
}

volatile int status;
volatile int IntCount;
static VL53L1_RangingMeasurementData_t RangingData;

//void setupLidar(void){
//  activateLidar();
//  HAL_Delay(5);
//  Dev->I2cHandle = &hi2c3;
//  Dev->I2cDevAddr = 0x52;
//  //printf("Autonomous Ranging Test\n");
//  status = VL53L1_WaitDeviceBooted(Dev);
//  status = VL53L1_DataInit(Dev);
//  status = VL53L1_StaticInit(Dev);
//  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT);
//  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 20000);
//  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 22);
//  status = VL53L1_StartMeasurement(Dev);
//  status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
//}

//void LidarMeasurement(void)
//{
//	//VL53L1_read_meas();
//	HAL_GPIO_TogglePin(LED_LAT_GPIO_Port, LED_LAT_Pin);
//	status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
//	HAL_GPIO_TogglePin(LED_LAT_GPIO_Port, LED_LAT_Pin);
//	//VL53L1_DEVICERESULTSLEVEL_FULL
//	//LidarMeasurementHandler(&temp_meas);
//	if(status == VL53L1_RANGESTATUS_RANGE_VALID){
//		//first measruement
//		if(measurement_hist == -1){
//			measurement_hist = RangingData.RangeMilliMeter;
//			LidarMeasurementHandler(&RangingData.RangeMilliMeter);
//		}
//		else if(abs(measurement_hist-RangingData.RangeMilliMeter) > THRESH_MEAS){
//			measurement_hist = RangingData.RangeMilliMeter;
//			LidarMeasurementHandler(&RangingData.RangeMilliMeter);
//		}
//		else{
//			measurement_hist = measurement_hist * ALPHA_MEAS + RangingData.RangeMilliMeter * BETA_MEAS;
//		}
//
//	}
//	status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
//	HAL_GPIO_TogglePin(LED_LAT_GPIO_Port, LED_LAT_Pin);
//	//VL53L1_clear_int_and_enable_range();
//	if(isLidarModeActive() == 0) lidarStop();
//}

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

