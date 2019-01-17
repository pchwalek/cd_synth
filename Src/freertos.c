/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "cap_touch.h"
#include "lidar.h"
#include "wave_synth.h"
#include "MMA8451.h"
#include "led_cntrl.h"
#include "res_touch.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//osTimerId capSampleTimerHandle;
osThreadId capSampleHandle;
osThreadId lidarMeasurementHandle;
osThreadId accSampleHandle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	/* Define a timer with “osTimerCallback” as callback process */
//	osTimerDef(capSampleTimer, Sample_Cap_Touch);
//    capSampleTimerHandle = osTimerCreate (osTimer(capSampleTimer), osTimerPeriodic, NULL);

  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //osThreadDef(cap_sample_thread, Sample_Cap_Touch, osPriorityNormal, 0, 128);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  osMutexDef (LED_mutex);    // Declare mutex
  LED_mutex_id = osMutexCreate(osMutex(LED_mutex));

  osMutexDef (I2C3_mutex);    // Declare mutex
  I2C3_mutex_id = osMutexCreate(osMutex(I2C3_mutex));
  osMutexWait(I2C3_mutex_id, 5);

  osSemaphoreDef(capSampleSemaphore);
  capSampleSemaphoreHandle = osSemaphoreCreate (osSemaphore(capSampleSemaphore), 1);
  osSemaphoreWait( capSampleSemaphoreHandle, 1);

  osSemaphoreDef(lidarSampleReadySemaphore);
  lidarSampleReadySemaphoreHandle = osSemaphoreCreate (osSemaphore(lidarSampleReadySemaphore), 1);
  osSemaphoreWait( lidarSampleReadySemaphoreHandle, 1);

  osSemaphoreDef(accSampleSemaphore);
  accSampleSemaphoreHandle = osSemaphoreCreate (osSemaphore(accSampleSemaphore), 1);
  osSemaphoreWait( accSampleSemaphoreHandle, 1);

  osTimerDef(accSampleTimer, accGiveSemaphore);
  accSampleTimerHandle = osTimerCreate(osTimer(accSampleTimer), osTimerPeriodic, (void *)0);

  osTimerDef(povExitTimer, enable_buttons);
  povExitTimerHandle = osTimerCreate(osTimer(povExitTimer), osTimerOnce, (void *)0);

  osThreadDef(lidarMeasurementTask, LidarMeasurement, osPriorityLow, 0, 256);
  lidarMeasurementHandle = osThreadCreate(osThread(lidarMeasurementTask), NULL);

  osThreadDef(capSampleTask, Sample_Cap_Touch, osPriorityNormal, 0, 256);
  capSampleHandle = osThreadCreate(osThread(capSampleTask), NULL);

  osThreadDef(accSampleTask, accelerometerThread, osPriorityLow, 0, 256);
  accSampleHandle = osThreadCreate(osThread(accSampleTask), NULL);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
