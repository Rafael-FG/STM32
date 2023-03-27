/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define TIM6_CLK 108000000
#define MAX_X 100

extern uint16_t send;
extern volatile uint8_t adc_print;
extern volatile uint8_t filter;
extern volatile uint8_t pid_print;
extern double Tsample;
extern double Tsample2;
extern double Tsample3;
extern float x[20];

typedef struct sampling {
	int x[MAX_X];
	int k, multiplier;
	uint32_t index, k_index;
	uint32_t units;
	char timeunit[6];
} sampling_s ;

typedef struct pid {
	int index, p_index;
	char timeunit[6];
	uint32_t units;
	uint8_t aut;
	float y, yr, e, sum_e, sum_e_bkp, e_ant, u_d, u, a, u_d_ant, kd, ki, kp, kd_h, kp_h, ki_h, y_ant, U_sat, U_sat_a, U_sat_b;
} pid_s ;

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
void MX_TIM9_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

void TIMER_2_Update(uint32_t reload);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
