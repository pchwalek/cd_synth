#include "hall_effect.h"
#include "stm32l4xx_hal.h"
#include "wave_synth.h"
#include "led_cntrl.h"

#include "stm32l4xx_hal_uart.h"
#include "usart.h"

//#define TICK_THRESH				2
//#define HALF_MAGNET_CNT			3

#define TICK_THRESH			6
#define HALF_MAGNET_CNT			7

//#define TIME_TO_ACTIVATE_POV

volatile int8_t intTracker 	= 0;
volatile uint8_t countDown 		= 0;
volatile uint8_t countUp		= 1;

volatile uint8_t POV_intTracker = 0;
volatile uint16_t rounds = 0;

volatile uint64_t RPR			= 0;

void resetIntTracker(void){
	POV_intTracker = 0;
}

void HALL_Handler(void){
	// don't need to read the pin state anymore since it will always be rising followed falling followed by....
//	if( HAL_GPIO_ReadPin(HALL_INT_GPIO_Port, HALL_INT_Pin) == GPIO_PIN_SET ){
//
//	}
//	else{
//
//	}
//	char temp_buf2[10];
//	rounds++;
//	itoa(rounds, temp_buf2, 10);
//	HAL_UART_Transmit(&huart3, (uint8_t*) temp_buf2, sizeof(temp_buf2), 10);

//	rounds++;
//	char temp_buf2[10];
//	itoa(rounds, temp_buf2, 10);
//	HAL_UART_Transmit(&huart3, (uint8_t*) temp_buf2, sizeof(temp_buf2), 10);
//	char str[5] = "\n\r";
//	HAL_UART_Transmit(&huart3, (uint8_t*) str, sizeof(str), 10);

	if(countUp){
		intTracker++;
	}
	else if(countDown){ //can remove this "if check" once confident code works properly
		intTracker--;
	}

	if(intTracker >= (ROTATION_STEPS-1)){
		countDown 	= 1;
		countUp 	= 0;
	}
	else if(intTracker <= 0){
		countDown 	= 0;
		countUp 	= 1;
	}

	setWavetableAmplitude(&intTracker);

	POV_intTracker++;

//	if(POV_intTracker == TICK_THRESH){
//		RPR = 2*(DWT->CYCCNT);
//		DWT->CYCCNT = 0; // reset the counter
//	}

	if(POV_intTracker == HALF_MAGNET_CNT){
		RPR = (DWT->CYCCNT);
		DWT->CYCCNT = 0; // reset the counter

		POV_intTracker = 1;
		POV_handler(RPR);
	}
}
