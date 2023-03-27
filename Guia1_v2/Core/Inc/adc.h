/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */

#define MAX_INDEX 46
#define ADC_ERROR ((uint32_t)ADC_CHANNEL_1)

extern uint32_t adcChannel;
extern char adc_buffer[10];
//extern double Tsample;
//extern uint8_t filter;

	
typedef struct signal {
	uint16_t x[MAX_INDEX];
	uint16_t	y[MAX_INDEX], k;
	uint32_t index, k_index, adc_channel;
	uint32_t units;
	char timeunit[6];
} signal_s ;

extern signal_s *adc_signal;


/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

uint32_t Discover_AdcChannel(uint8_t adc_channel);
int digital_lpf(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
