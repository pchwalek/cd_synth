#include "res_touch.h"
#include "led_cntrl.h"
#include "main.h"
#include "I2C.h"
#include "MMA8451.h"
#include "wave_synth.h"
#include "tim.h"
#include "filter.h"
#include "cap_touch.h"

#define LED_ON		1
#define LED_OFF		0

#define BUTTON_DEBOUNCE_THRESH_MS		1000

uint8_t accTimer = 0;

uint32_t button_1_timeout = 0;
uint32_t button_2_timeout = 0;
uint32_t button_3_timeout = 0;
uint32_t button_4_timeout = 0;
uint32_t button_5_timeout = 0;
uint32_t button_6_timeout = 0;
uint32_t button_7_timeout = 0;
uint32_t button_8_timeout = 0;
uint32_t button_9_timeout = 0;
uint32_t button_10_timeout = 0;

uint8_t button_10_debounce = 0;

uint8_t trigger_table	 = 0;
uint8_t first_pass		 = 0;

uint8_t temp_buffer[6];
uint8_t button_state = 1;

uint8_t button_10_state = 0;

uint8_t bit_crush = 0;
uint8_t button_9_state = 0;

uint8_t button_7_state = 0;
uint8_t button_8_state = 0;

void ResistiveTouchSampler(void){
//	uint8_t start_DMA = 0;
//	uint8_t start_DMA_2 = 0;
//	uint8_t button_4_debounce;

	if(isButtonEnabled() == 1){

		if(HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin) == GPIO_PIN_SET){
			if(LED_State(BUTTON_1_G_REG, BUTTON_1_G_PIN) == LED_OFF){
				if ( (HAL_GetTick() - button_1_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
					button_1_timeout = HAL_GetTick();
					Set_LED_Setting(BUTTON_1_G_REG, BUTTON_1_G_PIN, 1);
					Set_LED_Setting(BUTTON_2_G_REG, BUTTON_2_G_PIN, 0);
					Set_LED_Setting(BUTTON_3_G_REG, BUTTON_3_G_PIN, 0);
					//Set_LED(BUTTON_7_G_REG, BUTTON_7_G_PIN, 0);
					//Set_LED(BUTTON_8_G_REG, BUTTON_8_G_PIN, 0);
					setTable('S');
				}
			}
		}

		else if(HAL_GPIO_ReadPin(BUTTON_2_GPIO_Port, BUTTON_2_Pin) == GPIO_PIN_SET){
			if(LED_State(BUTTON_2_G_REG, BUTTON_2_G_PIN) == LED_OFF){
				if ( (HAL_GetTick() - button_2_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
					button_2_timeout = HAL_GetTick();
					Set_LED_Setting(BUTTON_1_G_REG, BUTTON_1_G_PIN, 0);
					Set_LED_Setting(BUTTON_2_G_REG, BUTTON_2_G_PIN, 1);
					Set_LED_Setting(BUTTON_3_G_REG, BUTTON_3_G_PIN, 0);
					//Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 0);
					//Set_LED_Setting(BUTTON_8_G_REG, BUTTON_8_G_PIN, 0);
					setTable('W');
				}
			}
		}

		else if(HAL_GPIO_ReadPin(BUTTON_3_GPIO_Port, BUTTON_3_Pin) == GPIO_PIN_SET){
			if(LED_State(BUTTON_3_G_REG, BUTTON_3_G_PIN) == LED_OFF){
				if ( (HAL_GetTick() - button_3_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
					button_3_timeout = HAL_GetTick();
					Set_LED_Setting(BUTTON_1_G_REG, BUTTON_1_G_PIN, 0);
					Set_LED_Setting(BUTTON_2_G_REG, BUTTON_2_G_PIN, 0);
					Set_LED_Setting(BUTTON_3_G_REG, BUTTON_3_G_PIN, 1);
					//Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 0);
					//Set_LED_Setting(BUTTON_8_G_REG, BUTTON_8_G_PIN, 0);
					setTable('Q');
				}
			}
		}

//		else if(HAL_GPIO_ReadPin(BUTTON_7_GPIO_Port, BUTTON_7_Pin) == GPIO_PIN_SET){
//			if(LED_State(BUTTON_7_G_REG, BUTTON_7_G_PIN) == LED_OFF){
//				if ( (HAL_GetTick() - button_7_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
//					button_7_timeout = HAL_GetTick();
//					Set_LED_Setting(BUTTON_1_G_REG, BUTTON_1_G_PIN, 0);
//					Set_LED_Setting(BUTTON_2_G_REG, BUTTON_2_G_PIN, 0);
//					Set_LED_Setting(BUTTON_3_G_REG, BUTTON_3_G_PIN, 0);
//					//Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 1);
//					//Set_LED_Setting(BUTTON_8_G_REG, BUTTON_8_G_PIN, 0);
//					setTable('T');
//					resetFilter();
//				}
//			}
//		}
//
//		else if(HAL_GPIO_ReadPin(BUTTON_8_GPIO_Port, BUTTON_8_Pin) == GPIO_PIN_SET){
//			if(LED_State(BUTTON_8_G_REG, BUTTON_8_G_PIN) == LED_OFF){
//				if ( (HAL_GetTick() - button_8_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
//					button_8_timeout = HAL_GetTick();
//					Set_LED_Setting(BUTTON_1_G_REG, BUTTON_1_G_PIN, 0);
//					Set_LED_Setting(BUTTON_2_G_REG, BUTTON_2_G_PIN, 0);
//					Set_LED_Setting(BUTTON_3_G_REG, BUTTON_3_G_PIN, 0);
//					//Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 0);
//					//Set_LED_Setting(BUTTON_8_G_REG, BUTTON_8_G_PIN, 1);
//					setTable('W');
//					resetFilter();
//				}
//			}
//		}

	//	if(HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin) == GPIO_PIN_SET){
	//		if(LED_State(BUTTON_1_R_REG, BUTTON_1_R_PIN) == LED_OFF){
	//			Set_LED_Setting(BUTTON_1_R_REG, BUTTON_1_R_PIN, 1);
	//			start_DMA = 1;
	//		}
	//
	////			index_1++;
	////			if(index_1 > 255){
	////				index_1 = 0;
	////			}
	//
	//	}
	//	else{
	//		if(LED_State(BUTTON_1_R_REG, BUTTON_1_R_PIN) == LED_ON){
	//			Set_LED_Setting(BUTTON_1_R_REG, BUTTON_1_R_PIN, 0);
	//		}
	////		if(index_1 >= 0){
	////			index_1 = -1;
	////		}
	//	}
	//
	//	if(HAL_GPIO_ReadPin(BUTTON_2_GPIO_Port, BUTTON_2_Pin) == GPIO_PIN_SET){
	//		if(LED_State(BUTTON_2_R_REG, BUTTON_2_R_PIN) == LED_OFF){
	//			Set_LED_Setting(BUTTON_2_R_REG, BUTTON_2_R_PIN, 1);
	//			start_DMA = 1;
	//		}
	//	}
	//	else{
	//		if(LED_State(BUTTON_2_R_REG, BUTTON_2_R_PIN) == LED_ON){
	//			Set_LED_Setting(BUTTON_2_R_REG, BUTTON_2_R_PIN, 0);
	//		}
	//	}
	//
	//	if(HAL_GPIO_ReadPin(BUTTON_3_GPIO_Port, BUTTON_3_Pin) == GPIO_PIN_SET){
	//		if(LED_State(BUTTON_3_R_REG, BUTTON_3_R_PIN) == LED_OFF){
	//			Set_LED_Setting(BUTTON_3_R_REG, BUTTON_3_R_PIN, 1);
	//			start_DMA = 1;
	//		}
	//	}
	//	else{
	//		if(LED_State(BUTTON_3_R_REG, BUTTON_3_R_PIN) == LED_ON){
	//			Set_LED_Setting(BUTTON_3_R_REG, BUTTON_3_R_PIN, 0);
	//		}
	//	}


		// make button 4 sticky (yes, sticky is an engineering term)
		if(HAL_GPIO_ReadPin(BUTTON_4_GPIO_Port, BUTTON_4_Pin) == GPIO_PIN_SET){
			if ( (HAL_GetTick() - button_4_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
				button_4_timeout = HAL_GetTick();

				if(LED_State(BUTTON_4_G_REG, BUTTON_4_G_PIN) == LED_OFF){
						Set_LED_Setting(BUTTON_4_G_REG, BUTTON_4_G_PIN, 1);
						activateLowpassFilter(1);
				}

				else if(LED_State(BUTTON_4_G_REG, BUTTON_4_G_PIN) == LED_ON){
						Set_LED_Setting(BUTTON_4_G_REG, BUTTON_4_G_PIN, 0);
						activateLowpassFilter(0);
				}
			}
		}

	//	else{
	//		if(button_4_debounce == 1) button_4_debounce = 0;
	//	}

		if(HAL_GPIO_ReadPin(BUTTON_5_GPIO_Port, BUTTON_5_Pin) == GPIO_PIN_SET){
			if ( (HAL_GetTick() - button_5_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
				button_5_timeout = HAL_GetTick();

				if(LED_State(BUTTON_5_G_REG, BUTTON_5_G_PIN) == LED_OFF){

					Set_LED_Setting(BUTTON_5_G_REG, BUTTON_5_G_PIN, 1);
					Set_LED_Setting(BUTTON_6_G_REG, BUTTON_6_G_PIN, 0);
				}
			}
		}
	//	else{
	//		if(LED_State(BUTTON_5_G_REG, BUTTON_5_G_PIN) == LED_ON){
	//			Set_LED_Setting(BUTTON_5_G_REG, BUTTON_5_G_PIN, 0);
	//		}
	//	}

		if(HAL_GPIO_ReadPin(BUTTON_6_GPIO_Port, BUTTON_6_Pin) == GPIO_PIN_SET){
			if(LED_State(BUTTON_6_G_REG, BUTTON_6_G_PIN) == LED_OFF){
				if ( (HAL_GetTick() - button_6_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
					button_6_timeout = HAL_GetTick();
					Set_LED_Setting(BUTTON_6_G_REG, BUTTON_6_G_PIN, 1);
					Set_LED_Setting(BUTTON_5_G_REG, BUTTON_5_G_PIN, 0);
				}
			}
		}
	//	else{
	//		if(LED_State(BUTTON_6_G_REG, BUTTON_6_G_PIN) == LED_ON){
	//			Set_LED_Setting(BUTTON_6_G_REG, BUTTON_6_G_PIN, 0);
	//		}
	//	}
	//
	//	if(HAL_GPIO_ReadPin(BUTTON_7_GPIO_Port, BUTTON_7_Pin) == GPIO_PIN_SET){
	//		if(LED_State(BUTTON_7_R_REG, BUTTON_7_R_PIN) == LED_OFF){
	//			Set_LED_Setting(BUTTON_7_R_REG, BUTTON_7_R_PIN, 1);
	//			start_DMA = 1;
	//		}
	//	}
	//	else{
	//		if(LED_State(BUTTON_7_R_REG, BUTTON_7_R_PIN) == LED_ON){
	//			Set_LED_Setting(BUTTON_7_R_REG, BUTTON_7_R_PIN, 0);
	//		}
	//	}
	//
	//	if(HAL_GPIO_ReadPin(BUTTON_8_GPIO_Port, BUTTON_8_Pin) == GPIO_PIN_SET){
	//		if(LED_State(BUTTON_8_R_REG, BUTTON_8_R_PIN) == LED_OFF){
	//			Set_LED_Setting(BUTTON_8_R_REG, BUTTON_8_R_PIN, 1);
	//			start_DMA = 1;
	//		}
	//	}
	//	else{
	//		if(LED_State(BUTTON_8_R_REG, BUTTON_8_R_PIN) == LED_ON){
	//			Set_LED_Setting(BUTTON_8_R_REG, BUTTON_8_R_PIN, 0);
	//		}
	//	}

//		if(HAL_GPIO_ReadPin(BUTTON_7_GPIO_Port, BUTTON_7_Pin) == GPIO_PIN_SET){
//		  if ( (HAL_GetTick() - button_7_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
//		    button_7_timeout = HAL_GetTick();
//
//		    button_7_state++;
//
//		    if(button_7_state == 1){
//		      Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 1);
//		      setPreWave(1);
//
//		    }
//		    else if(button_7_state == 2){
//		      Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 1);
//		      Set_LED_Setting(BUTTON_7_R_REG, BUTTON_7_R_PIN, 1);
//		      setPreWave(2);
//		    }
//		    else if(button_7_state == 3){
//		      Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 0);
//		      Set_LED_Setting(BUTTON_7_R_REG, BUTTON_7_R_PIN, 1);
//		      setPreWave(3);
//		    }
//		    else if(button_7_state == 4){
//		      Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 0);
//		      Set_LED_Setting(BUTTON_7_R_REG, BUTTON_7_R_PIN, 0);
//		      setPreWave(0);
//		      button_7_state = 0;
//		    }
//
//		  }
//		}

		if(HAL_GPIO_ReadPin(BUTTON_7_GPIO_Port, BUTTON_7_Pin) == GPIO_PIN_SET){
		  if ( (HAL_GetTick() - button_7_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
		    button_7_timeout = HAL_GetTick();

		    button_7_state++;

		    if(button_7_state == 1){
		      Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 1);
		      startRecording();
		      //setPreWave(1);

		    }
		    else if(button_7_state == 2){
		      Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 0);
		      Set_LED_Setting(BUTTON_7_R_REG, BUTTON_7_R_PIN, 1);
		      stopRecording();
		    }
		    else if(button_7_state == 3){
		      Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 1);
		      Set_LED_Setting(BUTTON_7_R_REG, BUTTON_7_R_PIN, 1);
		      startPlayback();
		    }
		    else if(button_7_state == 4){
		      Set_LED_Setting(BUTTON_7_G_REG, BUTTON_7_G_PIN, 0);
		      Set_LED_Setting(BUTTON_7_R_REG, BUTTON_7_R_PIN, 0);
		      //setPreWave(0);
		      stopPlayback();
		      button_7_state = 0;
		    }

		  }
		}

		if(HAL_GPIO_ReadPin(BUTTON_8_GPIO_Port, BUTTON_8_Pin) == GPIO_PIN_SET){
		  if ( (HAL_GetTick() - button_8_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
		    button_8_timeout = HAL_GetTick();

		    button_8_state++;

		    if(button_8_state == 1){
		      Set_LED_Setting(BUTTON_8_G_REG, BUTTON_8_G_PIN, 1);
		      setPostWave(1);

		    }
		    else if(button_8_state == 2){
		      Set_LED_Setting(BUTTON_8_G_REG, BUTTON_8_G_PIN, 1);
		      Set_LED_Setting(BUTTON_8_R_REG, BUTTON_8_R_PIN, 1);
		      setPostWave(2);
		    }
		    else if(button_8_state == 3){
		      Set_LED_Setting(BUTTON_8_G_REG, BUTTON_8_G_PIN, 0);
		      Set_LED_Setting(BUTTON_8_R_REG, BUTTON_8_R_PIN, 1);
		      setPostWave(3);
		    }
		    else if(button_8_state == 4){
		      Set_LED_Setting(BUTTON_8_G_REG, BUTTON_8_G_PIN, 0);
		      Set_LED_Setting(BUTTON_8_R_REG, BUTTON_8_R_PIN, 0);
		      setPostWave(0);
		      button_8_state = 0;
		    }

		  }
		}


//		if(HAL_GPIO_ReadPin(BUTTON_9_GPIO_Port, BUTTON_9_Pin) == GPIO_PIN_SET){
//			if ( (HAL_GetTick() - button_9_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
//				button_9_timeout = HAL_GetTick();
//				if(LED_State(BUTTON_9_G_REG, BUTTON_9_G_PIN) == LED_OFF){
//					Set_LED_Setting(BUTTON_9_G_REG, BUTTON_9_G_PIN, 1);
//					activateFilter(1);
//					trigger_table = 1;
//					first_pass = 1;
//				}
//				else{
//					Set_LED_Setting(BUTTON_9_G_REG, BUTTON_9_G_PIN, 0);
//					Set_LED_Setting(BUTTON_10_G_REG, BUTTON_10_G_PIN, 0);
//					Set_LED_Setting(BUTTON_10_R_REG, BUTTON_10_R_PIN, 0);
//					trigger_table = 0;
//					activateFilter(0);
//				}
//			}
//		}
//
//		if(trigger_table == 1){
//			if (HAL_GPIO_ReadPin(BUTTON_10_GPIO_Port, BUTTON_10_Pin) == GPIO_PIN_SET || (first_pass == 1) ){
//				first_pass = 0;
//
//				if ( (HAL_GetTick() - button_10_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
//					button_10_timeout = HAL_GetTick();
//
//					if(LED_State(BUTTON_10_G_REG, BUTTON_10_G_PIN) == LED_OFF){
//						Set_LED_Setting(BUTTON_10_R_REG, BUTTON_10_R_PIN, 0);
//						Set_LED_Setting(BUTTON_10_G_REG, BUTTON_10_G_PIN, 1);
//						incrementTable();
//					}
//
//					else if(LED_State(BUTTON_10_R_REG, BUTTON_10_R_PIN) == LED_OFF){
//						Set_LED_Setting(BUTTON_10_R_REG, BUTTON_10_R_PIN, 1);
//						Set_LED_Setting(BUTTON_10_G_REG, BUTTON_10_G_PIN, 0);
//						incrementTable();
//					}
//				}
//			}
//		}

		if(HAL_GPIO_ReadPin(BUTTON_9_GPIO_Port, BUTTON_9_Pin) == GPIO_PIN_SET){
		  if ( (HAL_GetTick() - button_9_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
		    button_9_timeout = HAL_GetTick();

		    button_9_state++;
		    bit_crush++;

		    if(button_9_state == 1){
		      Set_LED_Setting(BUTTON_9_G_REG, BUTTON_9_G_PIN, 1);
		      bit_crush = 4;
		    }
		    else if(button_9_state == 2){
		      Set_LED_Setting(BUTTON_9_G_REG, BUTTON_9_G_PIN, 1);
		      Set_LED_Setting(BUTTON_9_R_REG, BUTTON_9_R_PIN, 1);
		      bit_crush = 6;
		    }
		    else if(button_9_state == 3){
		      Set_LED_Setting(BUTTON_9_G_REG, BUTTON_9_G_PIN, 0);
		      Set_LED_Setting(BUTTON_9_R_REG, BUTTON_9_R_PIN, 1);
		      bit_crush = 8;
		    }
		    else{
		      Set_LED_Setting(BUTTON_9_G_REG, BUTTON_9_G_PIN, 0);
		      Set_LED_Setting(BUTTON_9_R_REG, BUTTON_9_R_PIN, 0);
		      bit_crush = 0;
		      button_9_state = 0;
		    }
//		    else if(bit_crush == 5){
//		      Set_LED_Setting(BUTTON_9_G_REG, BUTTON_9_G_PIN, 0);
//		      Set_LED_Setting(BUTTON_9_R_REG, BUTTON_9_R_PIN, 0);
//		      changeQ(0.8);
//		      bit_crush = 0;
//		    }

		  }
		}

		if(HAL_GPIO_ReadPin(BUTTON_10_GPIO_Port, BUTTON_10_Pin) == GPIO_PIN_SET){
		  if ( (HAL_GetTick() - button_10_timeout) > BUTTON_DEBOUNCE_THRESH_MS){
		    button_10_timeout = HAL_GetTick();

		    button_10_state++;

		    if(button_10_state == 1){
		      Set_LED_Setting(BUTTON_10_G_REG, BUTTON_10_G_PIN, 1);
		      changeQ(1);

		    }
		    else if(button_10_state == 2){
		      Set_LED_Setting(BUTTON_10_G_REG, BUTTON_10_G_PIN, 1);
		      Set_LED_Setting(BUTTON_10_R_REG, BUTTON_10_R_PIN, 1);
		      changeQ(1.5);
		    }
		    else if(button_10_state == 3){
		      Set_LED_Setting(BUTTON_10_G_REG, BUTTON_10_G_PIN, 0);
		      Set_LED_Setting(BUTTON_10_R_REG, BUTTON_10_R_PIN, 1);
		      changeQ(2);
		    }
		    else if(button_10_state == 4){
		      Set_LED_Setting(BUTTON_10_G_REG, BUTTON_10_G_PIN, 0);
		      Set_LED_Setting(BUTTON_10_R_REG, BUTTON_10_R_PIN, 0);
		      changeQ(3);
		      button_10_state = 0;
		    }

		  }
		}




	//	if(start_DMA){
	//		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)sine_wave_array, 32, DAC_ALIGN_12B_R);
	//	}
	//	else if(start_DMA_2){
	//		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)waveTable_1, 255, DAC_ALIGN_12B_R);
	//	}
	//	else
	//	{
	//		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
	//	}
		//osDelay(2);
		transmitToBuffer();

		    taskENTER_CRITICAL();

		buttonStateMachine();
		    taskEXIT_CRITICAL();

	}
}

uint8_t getBitCrush(void){
  return bit_crush;
}

void buttonStateMachine(void){
	// if accelerometer is enabled, start FreeRTOS Thread Timer
	if( (LED_State(BUTTON_4_G_REG, BUTTON_4_G_PIN) == LED_ON) && (accTimer == 0)){
		osTimerStart(accSampleTimerHandle, 20);
		accTimer = 1;
	}else if ( (LED_State(BUTTON_4_G_REG, BUTTON_4_G_PIN) == LED_OFF) && (accTimer == 1) ){
		osTimerStop(accSampleTimerHandle);
		accTimer = 0;

	}

	if(LED_State(BUTTON_5_G_REG, BUTTON_5_G_PIN) == LED_ON){
		turnOnLidarSounds();
		osTimerStop(capSampleTimerHandle);
	}
	else if(LED_State(BUTTON_6_G_REG, BUTTON_6_G_PIN) == LED_ON){
		turnOnCapSounds();
		osTimerStart(capSampleTimerHandle, 20);

	}else{
		turnOffSounds();
		osTimerStop(capSampleTimerHandle);
	}
}

void disable_buttons(void){
	if(isButtonEnabled() == 1){
		button_state = 0;
		temp_buffer[0] = LED_SETTINGS[0];
		temp_buffer[1] = LED_SETTINGS[1];
		temp_buffer[2] = LED_SETTINGS[2];
		temp_buffer[3] = LED_SETTINGS[3];
		temp_buffer[4] = LED_SETTINGS[4];
		temp_buffer[5] = LED_SETTINGS[5];
		//HAL_TIM_Base_Start_IT(&htim4);
		osTimerStart(povExitTimerHandle, 400);
	}else{
		//restart timer
		//htim4.Instance->CNT = 0;
//		HAL_TIM_Base_Stop_IT(&htim4);
//		HAL_TIM_Base_Start_IT(&htim4);
	    osTimerStop(povExitTimerHandle);
	    osTimerStart(povExitTimerHandle, 400);

	}
}

void enable_buttons(void){
	//taskENTER_CRITICAL();


	//HAL_TIM_Base_Stop_IT(&htim4);
	LED_SETTINGS[0] = temp_buffer[0];
	LED_SETTINGS[1] = temp_buffer[1];
	LED_SETTINGS[2] = temp_buffer[2];
	LED_SETTINGS[3] = temp_buffer[3];
	LED_SETTINGS[4] = temp_buffer[4];
	LED_SETTINGS[5] = temp_buffer[5];
	transmitToBuffer();
	button_state = 1;
	//taskEXIT_CRITICAL();
}

uint8_t isButtonEnabled(void){
	return button_state;
}
