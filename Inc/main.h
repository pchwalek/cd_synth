/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_3_Pin GPIO_PIN_2
#define BUTTON_3_GPIO_Port GPIOE
#define BUTTON_3_EXTI_IRQn EXTI2_IRQn
#define BUTTON_8_Pin GPIO_PIN_3
#define BUTTON_8_GPIO_Port GPIOE
#define BUTTON_8_EXTI_IRQn EXTI3_IRQn
#define POWER_SWITCH_Pin GPIO_PIN_4
#define POWER_SWITCH_GPIO_Port GPIOE
#define BUTTON_2_Pin GPIO_PIN_13
#define BUTTON_2_GPIO_Port GPIOC
#define BUTTON_2_EXTI_IRQn EXTI15_10_IRQn
#define HALL_OUTPUT_Pin GPIO_PIN_2
#define HALL_OUTPUT_GPIO_Port GPIOC
#define LED_MOSI_Pin GPIO_PIN_3
#define LED_MOSI_GPIO_Port GPIOC
#define BUTTON_7_Pin GPIO_PIN_0
#define BUTTON_7_GPIO_Port GPIOA
#define BUTTON_7_EXTI_IRQn EXTI0_IRQn
#define BUTTON_1_Pin GPIO_PIN_1
#define BUTTON_1_GPIO_Port GPIOA
#define BUTTON_1_EXTI_IRQn EXTI1_IRQn
#define LED_LAT_Pin GPIO_PIN_2
#define LED_LAT_GPIO_Port GPIOA
#define LED_SS_Pin GPIO_PIN_8
#define LED_SS_GPIO_Port GPIOE
#define LED_PWM_Pin GPIO_PIN_9
#define LED_PWM_GPIO_Port GPIOE
#define LED_SCK_Pin GPIO_PIN_13
#define LED_SCK_GPIO_Port GPIOB
#define BT_CMD_Pin GPIO_PIN_9
#define BT_CMD_GPIO_Port GPIOD
#define BUTTON_6_Pin GPIO_PIN_10
#define BUTTON_6_GPIO_Port GPIOD
#define BUTTON_6_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_10_Pin GPIO_PIN_11
#define BUTTON_10_GPIO_Port GPIOD
#define BUTTON_10_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_5_Pin GPIO_PIN_12
#define BUTTON_5_GPIO_Port GPIOD
#define BUTTON_5_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_4_Pin GPIO_PIN_14
#define BUTTON_4_GPIO_Port GPIOD
#define BUTTON_4_EXTI_IRQn EXTI15_10_IRQn
#define VL_XSHUT_Pin GPIO_PIN_15
#define VL_XSHUT_GPIO_Port GPIOD
#define BUTTON_9_Pin GPIO_PIN_7
#define BUTTON_9_GPIO_Port GPIOC
#define BUTTON_9_EXTI_IRQn EXTI9_5_IRQn
#define VL_INT_Pin GPIO_PIN_8
#define VL_INT_GPIO_Port GPIOA
#define VL_INT_EXTI_IRQn EXTI9_5_IRQn
#define HALL_INT_Pin GPIO_PIN_15
#define HALL_INT_GPIO_Port GPIOA
#define HALL_INT_EXTI_IRQn EXTI15_10_IRQn
#define HALL_CNTRL_Pin GPIO_PIN_6
#define HALL_CNTRL_GPIO_Port GPIOD
#define SD_CARD_EN_Pin GPIO_PIN_7
#define SD_CARD_EN_GPIO_Port GPIOD
#define CAP_ALERT_Pin GPIO_PIN_5
#define CAP_ALERT_GPIO_Port GPIOB
#define CAP_ALERT_EXTI_IRQn EXTI9_5_IRQn
#define BT_EN_Pin GPIO_PIN_0
#define BT_EN_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
