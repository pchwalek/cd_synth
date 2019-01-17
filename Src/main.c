/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lidar.h"
#include "res_touch.h"
#include "cap_touch.h"
#include "led_cntrl.h"
#include "wave_synth.h"
#include "MMA8451.h"
#include "stdlib.h"
//#include "tables.h"
#include "filter.h"
#include "hall_effect.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HALL_UPPER_THRESH	550
#define HALL_LOWER_THRESH	430
#define ADC_SAMPLE_TIME		0.000208
#define RPS_LED_THRESH		0.080

//define below  to enable BT_UART over FTDI connection
//#define BT_UART				1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t byte1;
volatile uint8_t byte3;
//char str[30] = "C,C95A02C57C49\n";
volatile uint16_t POV_map3 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_DAC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART3_UART_Init();
  MX_SDMMC1_SD_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_COMP1_Init();
  /* USER CODE BEGIN 2 */

  //turn on 3.3V power for sensors
  HAL_GPIO_WritePin(POWER_SWITCH_GPIO_Port, POWER_SWITCH_Pin, GPIO_PIN_SET);

  //enable BT module
  //HAL_GPIO_WritePin(BT_CMD_GPIO_Port, BT_CMD_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BT_EN_GPIO_Port, BT_EN_Pin, GPIO_PIN_SET);

  //  HAL_GPIO_TogglePin(LED_LAT_GPIO_Port, LED_LAT_Pin);
  activateLidar();

  // short delay to give Lidar some time to initialize
  HAL_Delay(100);

  // enable all shift registers
  //HAL_GPIO_WritePin(LED_PWM_GPIO_Port, LED_PWM_Pin, GPIO_PIN_RESET);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  Setup_Cap_Touch();

  // below is needed to ensure proper rounds-per-second is calculated for the POV display
  /* DWT struct is defined inside the core_cm4.h file */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  //DWT->LAR = 0xC5ACCE55;
  DWT->CYCCNT = 0; // reset the counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // enable the counter

  HAL_TIM_Base_Start(&htim6);

  // DAC DMA
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  //    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  //    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0x100);

  // prep an initial buffer for DAC
  prepBuffer(&hdac1);

  // turn off all LEDs (ensures no weird states on startup)
  Flush_LEDS();

#ifdef BT_UART
  HAL_GPIO_WritePin(BT_CMD_GPIO_Port, BT_CMD_Pin, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart3, &byte3, 1);
  HAL_UART_Receive_IT(&huart1, &byte1, 1);
#endif

  //    // set hall effect to HIGH (its 6.25kHz if set high and 24Hz when low)
  HAL_GPIO_WritePin(HALL_CNTRL_GPIO_Port, HALL_CNTRL_Pin, GPIO_PIN_SET);
  //
  //    // enable ADC
  //    ADC_Enable(&hadc1);
  //    HAL_ADC_Start_IT(&hadc1);

  //initFilter();

  // start comparator for HALL effect sensor
  HAL_COMP_Start(&hcomp1);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLP;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//volatile uint32_t temp_adc;
//char temp_buf2[10];
//uint32_t temp_time;


//volatile uint32_t curr_ADC_sample = 0;
//volatile uint32_t prev_ADC_sample = 0;
//
//
//volatile uint8_t quarter_spin_up 	= 	0;
//volatile uint8_t quarter_spin_down 	= 	0;
//volatile float RPS 					= 	0;

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	curr_ADC_sample++;
//  if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))
//    {
//
//	temp_adc = HAL_ADC_GetValue(hadc);
//	if(temp_adc > HALL_UPPER_THRESH){
//		if(quarter_spin_up == 0){
//			quarter_spin_up = 1;
//
//			RPS = (curr_ADC_sample - prev_ADC_sample) * ((float) ADC_SAMPLE_TIME) * 4;
//			prev_ADC_sample = curr_ADC_sample;
//			RPS_LED();
//		}
//
//	}
//	else if (temp_adc < HALL_LOWER_THRESH){
//		if(quarter_spin_down == 0){
//			quarter_spin_down = 1;
//
//			RPS = (curr_ADC_sample - prev_ADC_sample) * ((float) ADC_SAMPLE_TIME) * 4;
//			prev_ADC_sample = curr_ADC_sample;
//			RPS_LED();
//		}
//
//	}
//	else{
//		quarter_spin_up = 0;
//		quarter_spin_down = 0;
//	}
//
//	itoa(temp_adc, temp_buf2, 10);
//	HAL_UART_Transmit(&huart3, (uint8_t*) temp_buf2, sizeof(temp_buf2), 10);
//	char str[5] = "\n\r";
//	HAL_UART_Transmit(&huart3, (uint8_t*) str, sizeof(str), 10);
//    }
//}


//void RPS_LED(void){
//	if(RPS < RPS_LED_THRESH){
//		ledOut2(1);
//	}
//	else if(RPS < (2*RPS_LED_THRESH)){
//		ledOut2(2);
//	}
//	else if(RPS < (3*RPS_LED_THRESH)){
//		ledOut2(3);
//		}
//	else if(RPS < (4*RPS_LED_THRESH)){
//		ledOut2(4);
//		}
//	else if(RPS < (5*RPS_LED_THRESH)){
//		ledOut2(5);
//		}
//	else if(RPS < (6*RPS_LED_THRESH)){
//		ledOut2(6);
//		}
//	else if(RPS < (7*RPS_LED_THRESH)){
//		ledOut2(7);
//		}
//	else if(RPS < (8*RPS_LED_THRESH)){
//		ledOut2(8);
//		}
//	else if(RPS < (9*RPS_LED_THRESH)){
//		ledOut2(9);
//		}
//	else if(RPS < (10*RPS_LED_THRESH)){
//		ledOut2(10);
//		}
//	else if(RPS < (11*RPS_LED_THRESH)){
//		ledOut2(11);
//		}
//	else if(RPS < (12*RPS_LED_THRESH)){
//		ledOut2(12);
//			}
//	else if(RPS < (13*RPS_LED_THRESH)){
//		ledOut2(13);
//			}
//	else if(RPS < (14*RPS_LED_THRESH)){
//		ledOut2(14);
//			}
//	else if(RPS < (15*RPS_LED_THRESH)){
//		ledOut2(15);
//	}
//}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp){
	HALL_Handler();
}


void ledOut2(int8_t shift){
	POV_map3 = 0x0001 << (( shift) % 15);
	POV_LEDs(POV_map3);
}

//volatile uint8_t LED_state = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  if(GPIO_Pin == CAP_ALERT_Pin){
	  //
	  if (isCapModeActive()) osSemaphoreRelease (capSampleSemaphoreHandle);
	  //if (isCapModeActive()) Sample_Cap_Touch();
	  //Sample_Cap_Touch();
  }
  else if(GPIO_Pin == VL_INT_Pin){
	  if(isLidarModeActive()) osSemaphoreRelease (lidarSampleReadySemaphoreHandle);

//	  if (isLidarModeActive()){
//		  HAL_GPIO_TogglePin(LED_LAT_GPIO_Port, LED_LAT_Pin);
//		  LidarMeasurement();
//	  }
	  //LidarMeasurement();
  }
//  else if(GPIO_Pin == HALL_INT_Pin){
//	  //LED_state++;
////	  Set_LED(BUTTON_3_G_REG, BUTTON_3_G_PIN, LED_state%2);
//	  //HALL_Handler();
//  }
  else{
	  ResistiveTouchSampler();
  }
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	HAL_GPIO_WritePin(LED_SS_GPIO_Port, LED_SS_Pin, GPIO_PIN_SET);

	if (LED_mutex_id != NULL)  {
		osMutexRelease(LED_mutex_id);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    /* Transmit one byte with 100 ms timeout */
    HAL_UART_Transmit(&huart1, &byte3, 1, 100);

    /* Receive one byte in interrupt mode */
    HAL_UART_Receive_IT(&huart3, &byte3, 1);

    //HAL_UART_Transmit(&huart3, "transmitting to BT module\n", sizeof("transmitting to BT module\n"), 100);

//    if (byte3 == '?'){
////        	HAL_UART_Transmit(&huart3, "  --MENU--  \n", sizeof("  --MENU--  \n"), 100);
////        	HAL_UART_Transmit(&huart3, "m: enter BT MAC\n", sizeof("m: enter BT MAC\n"), 100);
//        }
//
////    if (byte == 'm'){
////    	HAL_UART_Transmit(&huart3, "RECEIVED M\n", sizeof("RECEIVED M\n"), 100);
////    }
  }

  if (huart->Instance == USART1)
   {
     /* Transmit one byte with 100 ms timeout */
     HAL_UART_Transmit(&huart3, &byte1, 1, 100);

     /* Receive one byte in interrupt mode */
     HAL_UART_Receive_IT(&huart1, &byte1, 1);

//     if (byte1 == '?'){
// //        	HAL_UART_Transmit(&huart3, "  --MENU--  \n", sizeof("  --MENU--  \n"), 100);
// //        	HAL_UART_Transmit(&huart3, "m: enter BT MAC\n", sizeof("m: enter BT MAC\n"), 100);
//         }
//
////     if (byte == 'm'){
////     	HAL_UART_Transmit(&huart3, "RECEIVED M\n", sizeof("RECEIVED M\n"), 100);
////     }
   }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM3) {
    POV_Update();
  }
  else if (htim->Instance == TIM4) {
    enable_buttons();
  }
//  else if (htim->Instance == TIM6) {
//	  HAL_GPIO_TogglePin(LED_LAT_GPIO_Port, LED_LAT_Pin);
//    }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
