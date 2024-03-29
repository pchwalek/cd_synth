#include "cap_touch.h"
#include "led_cntrl.h"
#include "I2C.h"
#include "wave_synth.h"
#include "res_touch.h"

#include "usart.h"
//#include "freertos.c"
//#include "stm32l4xx_hal_dac.h"

/**************** defines ***************************/


/**************** variables ***************************/
volatile uint8_t cap_read[2];
int8_t* new_temp;
uint8_t temp[14];

uint8_t leftTouchDebounce = 0;
uint8_t rightTouchDebounce = 0;

uint8_t packet;
const uint8_t packet_null = 0x00;

/**************** functions ***************************/
void Setup_Cap_Touch(void){


	// ungroup all CAP sensors to work individually
	packet = 0x02;
	HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, CNFG_REG_4, 1, &packet, 1, 100);

	// device will not block multiple touches
	packet = 0x04;
	HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, MULT_TOUCH_REG, 1, &packet, 1, 100);

	// disable auto-calibration
	packet = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, CALIBRATION_REG, 1, &packet, 1, 100);

	// sensitivity control
	packet = 0x7F & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, DATA_SENS_REG, 1, &packet, 1, 100);

	// setting button 1 threshold sets all
//	packet = 0x93 | 0x40;
//	HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, RECAL_REG, 1, &packet, 1, 100);

	packet = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, MAIN_STATUS, 1, &packet, 1, 100);

//	packet = 0x04 | 0x01;
//HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, 0x20, 1, &packet, 1, 100);

//    packet = 0xFF;
//    HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, 0x4E, 1, &packet, 1, 100);

//    packet = 0xFF;
//        HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, 0x28, 1, &packet, 1, 100);

//    packet = 0x03;
//    HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, 0x4E, 1, &packet, 1, 100);

	// disable auto-calibration
//	while(1){
//		packet = 0x00;
//		HAL_I2C_Mem_Write(&hi2c1, CAP1214_ADDR<<1, 0x00, 1, &packet, 1, 1);
//
//		HAL_I2C_Mem_Read(&hi2c1, CAP1214_ADDR<<1, SENSOR_1_DELTA_CNT, 1, &temp, 14, 1);
//	}
}


// read binary cap touch sense registors from CAP1214 IC
void Read_Cap_Touch(void){
	//uint8_t packet;

	HAL_I2C_Mem_Read_IT(&hi2c1, CAP1214_ADDR<<1, 0x03, 1, cap_read, 2);


//	HAL_I2C_Mem_Read(&hi2c1, CAP1214_ADDR<<1, 0x51, 1, temp, 1, 1);
//	new_temp = temp;

	//HAL_Delay(1);
	//Reset_Cap_INT();
}


void Reset_Cap_INT(void){
	HAL_I2C_Mem_Write_IT(&hi2c1, CAP1214_ADDR<<1, 0x00, 1, &packet_null, 1);
}

void capGiveSemaphore(void){
  osSemaphoreRelease (capSampleSemaphoreHandle);
}

//// reads cap touch sense binary values from CAP1214 IC and
////    turns on corresponding LEDs
void Sample_Cap_Touch(void){
//  uint8_t start_DMA = 0;
//  uint8_t start_DMA_2 = 0;

  //Reset_Cap_INT();

  switchOctave(4);

  while(1){
#ifdef DEBUG_PRINT
  HAL_UART_Transmit(&huart3, "i2c int\n\r", sizeof("i2c int passed\n\r"), 100);
#endif
    Reset_Cap_INT();
#ifdef DEBUG_PRINT
  HAL_UART_Transmit(&huart3, " passed\n\r", sizeof(" passed\n\r"), 100);
#endif
    osSemaphoreWait (capSampleSemaphoreHandle, osWaitForever);
#ifdef DEBUG_PRINT
  HAL_UART_Transmit(&huart3, "start_cap\n\r", sizeof("start_cap\n\r"), 100);
#endif
    //taskENTER_CRITICAL();
    Read_Cap_Touch();
    osDelay(2);
#ifdef DEBUG_PRINT
  HAL_UART_Transmit(&huart3, "i2c sample passed\n\r", sizeof("i2c sample passed\n\r"), 100);
#endif

    if(isButtonEnabled() == 1){
//      if( (cap_read[KEY_1_PORT] & KEY_1_PIN) == KEY_1_PIN){
//	  Set_LED(CAP_1_LED_PORT, CAP_1_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_1_LED_PORT, CAP_1_LED_PIN)){
//		      Set_LED(CAP_1_LED_PORT, CAP_1_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_2_PORT] & KEY_2_PIN) == KEY_2_PIN){
//	      Set_LED(CAP_2_LED_PORT, CAP_2_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_2_LED_PORT, CAP_2_LED_PIN)){
//		      Set_LED(CAP_2_LED_PORT, CAP_2_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_3_PORT] & KEY_3_PIN) == KEY_3_PIN){
//	      Set_LED(CAP_3_LED_PORT, CAP_3_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_3_LED_PORT, CAP_3_LED_PIN)){
//		      Set_LED(CAP_3_LED_PORT, CAP_3_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_4_PORT] & KEY_4_PIN) == KEY_4_PIN){
//	      Set_LED(CAP_4_LED_PORT, CAP_4_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_4_LED_PORT, CAP_4_LED_PIN)){
//		      Set_LED(CAP_4_LED_PORT, CAP_4_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_5_PORT] & KEY_5_PIN) == KEY_5_PIN){
//	      Set_LED(CAP_5_LED_PORT, CAP_5_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_5_LED_PORT, CAP_5_LED_PIN)){
//		      Set_LED(CAP_5_LED_PORT, CAP_5_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_6_PORT] & KEY_6_PIN) == KEY_6_PIN){
//	      Set_LED(CAP_6_LED_PORT, CAP_6_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_6_LED_PORT, CAP_6_LED_PIN)){
//		      Set_LED(CAP_6_LED_PORT, CAP_6_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_7_PORT] & KEY_7_PIN) == KEY_7_PIN){
//	      Set_LED(CAP_7_LED_PORT, CAP_7_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_7_LED_PORT, CAP_7_LED_PIN)){
//		      Set_LED(CAP_7_LED_PORT, CAP_7_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_8_PORT] & KEY_8_PIN) == KEY_8_PIN){
//	      Set_LED(CAP_8_LED_PORT, CAP_8_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_8_LED_PORT, CAP_8_LED_PIN)){
//		      Set_LED(CAP_8_LED_PORT, CAP_8_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_9_PORT] & KEY_9_PIN) == KEY_9_PIN){
//	      Set_LED(CAP_9_LED_PORT, CAP_9_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_9_LED_PORT, CAP_9_LED_PIN)){
//		      Set_LED(CAP_9_LED_PORT, CAP_9_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_10_PORT] & KEY_10_PIN) == KEY_10_PIN){
//	      Set_LED(CAP_10_LED_PORT, CAP_10_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_10_LED_PORT, CAP_10_LED_PIN)){
//		      Set_LED(CAP_10_LED_PORT, CAP_10_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_11_PORT] & KEY_11_PIN) == KEY_11_PIN){
//	      Set_LED(CAP_11_LED_PORT, CAP_11_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_11_LED_PORT, CAP_11_LED_PIN)){
//		      Set_LED(CAP_11_LED_PORT, CAP_11_LED_PIN, 0);
//	      }
//      }
//
//      if( (cap_read[KEY_12_PORT] & KEY_12_PIN) == KEY_12_PIN){
//	      Set_LED(CAP_12_LED_PORT, CAP_12_LED_PIN, 1);
//      }
//      else{
//	      if(LED_State(CAP_12_LED_PORT, CAP_12_LED_PIN)){
//		      Set_LED(CAP_12_LED_PORT, CAP_12_LED_PIN, 0);
//	      }
//      }

      if( (cap_read[KEY_1_PORT] & KEY_1_PIN) == KEY_1_PIN){
	  if( LED_State(CAP_1_LED_PORT, CAP_1_LED_PIN) != 1 ){
	      nullTracker(1);
	      Set_LED_Setting(CAP_1_LED_PORT, CAP_1_LED_PIN, 1);
	  }

      }
      else{
	      if(LED_State(CAP_1_LED_PORT, CAP_1_LED_PIN)){
		  resetFrequencyInd(1);
		  Set_LED_Setting(CAP_1_LED_PORT, CAP_1_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_2_PORT] & KEY_2_PIN) == KEY_2_PIN){
	  if( LED_State(CAP_2_LED_PORT, CAP_2_LED_PIN) != 1 ){
	  nullTracker(2);
	  Set_LED_Setting(CAP_2_LED_PORT, CAP_2_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_2_LED_PORT, CAP_2_LED_PIN)){
		  resetFrequencyInd(2);
		  Set_LED_Setting(CAP_2_LED_PORT, CAP_2_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_3_PORT] & KEY_3_PIN) == KEY_3_PIN){
	  if( LED_State(CAP_3_LED_PORT, CAP_3_LED_PIN) != 1 ){
	  	  nullTracker(3);
	  Set_LED_Setting(CAP_3_LED_PORT, CAP_3_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_3_LED_PORT, CAP_3_LED_PIN)){
		  resetFrequencyInd(3);
		  Set_LED_Setting(CAP_3_LED_PORT, CAP_3_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_4_PORT] & KEY_4_PIN) == KEY_4_PIN){
	  if( LED_State(CAP_4_LED_PORT, CAP_4_LED_PIN) != 1 ){
	  	  nullTracker(4);
	  Set_LED_Setting(CAP_4_LED_PORT, CAP_4_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_4_LED_PORT, CAP_4_LED_PIN)){
		  resetFrequencyInd(4);
		  Set_LED_Setting(CAP_4_LED_PORT, CAP_4_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_5_PORT] & KEY_5_PIN) == KEY_5_PIN){
	  if( LED_State(CAP_5_LED_PORT, CAP_5_LED_PIN) != 1 ){
	  	  nullTracker(5);
	  Set_LED_Setting(CAP_5_LED_PORT, CAP_5_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_5_LED_PORT, CAP_5_LED_PIN)){
		  resetFrequencyInd(5);
		  Set_LED_Setting(CAP_5_LED_PORT, CAP_5_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_6_PORT] & KEY_6_PIN) == KEY_6_PIN){
	  if( LED_State(CAP_6_LED_PORT, CAP_6_LED_PIN) != 1 ){
	  	  nullTracker(6);
	  Set_LED_Setting(CAP_6_LED_PORT, CAP_6_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_6_LED_PORT, CAP_6_LED_PIN)){
		  resetFrequencyInd(6);
		  Set_LED_Setting(CAP_6_LED_PORT, CAP_6_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_7_PORT] & KEY_7_PIN) == KEY_7_PIN){
	  if( LED_State(CAP_7_LED_PORT, CAP_7_LED_PIN) != 1 ){
	  	  nullTracker(7);
	  Set_LED_Setting(CAP_7_LED_PORT, CAP_7_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_7_LED_PORT, CAP_7_LED_PIN)){
		  resetFrequencyInd(7);
		  Set_LED_Setting(CAP_7_LED_PORT, CAP_7_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_8_PORT] & KEY_8_PIN) == KEY_8_PIN){
	  if( LED_State(CAP_8_LED_PORT, CAP_8_LED_PIN) != 1 ){
	  	  nullTracker(8);
	  Set_LED_Setting(CAP_8_LED_PORT, CAP_8_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_8_LED_PORT, CAP_8_LED_PIN)){
		  resetFrequencyInd(8);
		  Set_LED_Setting(CAP_8_LED_PORT, CAP_8_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_9_PORT] & KEY_9_PIN) == KEY_9_PIN){
	  if( LED_State(CAP_9_LED_PORT, CAP_9_LED_PIN) != 1 ){
	  	  nullTracker(9);
	  Set_LED_Setting(CAP_9_LED_PORT, CAP_9_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_9_LED_PORT, CAP_9_LED_PIN)){
		  resetFrequencyInd(9);
		  Set_LED_Setting(CAP_9_LED_PORT, CAP_9_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_10_PORT] & KEY_10_PIN) == KEY_10_PIN){
	  if( LED_State(CAP_10_LED_PORT, CAP_10_LED_PIN) != 1 ){
	  	  nullTracker(10);
	  Set_LED_Setting(CAP_10_LED_PORT, CAP_10_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_10_LED_PORT, CAP_10_LED_PIN)){
		  resetFrequencyInd(10);
		  Set_LED_Setting(CAP_10_LED_PORT, CAP_10_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_11_PORT] & KEY_11_PIN) == KEY_11_PIN){
	  if( LED_State(CAP_11_LED_PORT, CAP_11_LED_PIN) != 1 ){
	  	  nullTracker(11);
	  Set_LED_Setting(CAP_11_LED_PORT, CAP_11_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_11_LED_PORT, CAP_11_LED_PIN)){
		  resetFrequencyInd(11);
		  Set_LED_Setting(CAP_11_LED_PORT, CAP_11_LED_PIN, 0);
	      }
      }

      if( (cap_read[KEY_12_PORT] & KEY_12_PIN) == KEY_12_PIN){
	  if( LED_State(CAP_12_LED_PORT, CAP_12_LED_PIN) != 1 ){
	  	  nullTracker(12);
	  Set_LED_Setting(CAP_12_LED_PORT, CAP_12_LED_PIN, 1);
	  }
      }
      else{
	      if(LED_State(CAP_12_LED_PORT, CAP_12_LED_PIN)){
		  resetFrequencyInd(12);
		  Set_LED_Setting(CAP_12_LED_PORT, CAP_12_LED_PIN, 0);
	      }
      }

      transmitToBuffer();
      //taskEXIT_CRITICAL();

      if( (cap_read[LEFT_BUTTON_PORT] & LEFT_BUTTON_PIN) == LEFT_BUTTON_PIN){
	      if(leftTouchDebounce == 0){
		      decrementOctave();
		      leftTouchDebounce = 1;
	      }
      }
      else{
	      if(leftTouchDebounce){
		      leftTouchDebounce = 0;
	      }
      }

      if( (cap_read[RIGHT_BUTTON_PORT] & RIGHT_BUTTON_PIN) == RIGHT_BUTTON_PIN){
	      if(rightTouchDebounce == 0){
		      incrementOctave();
		      rightTouchDebounce = 1;
	      }
      }
      else{
	      if(rightTouchDebounce){
		      rightTouchDebounce = 0;
	      }
      }
    }
#ifdef DEBUG_PRINT
  HAL_UART_Transmit(&huart3, "stop_cap\n\r", sizeof("stop_cap\n\r"), 100);
#endif
  }
}
