/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ctype.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern signal_s *adc_signal;
extern sampling_s *sampling;
extern pid_s *pid;
extern uint8_t sampling_print;
extern char sampling_buffer[20];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define MR 	0x4D52 												
//#define MW 	0x4D57		
//#define MI 	0x4D49	
//#define MO 	0x4D4F		
//#define RD 	0x5244		
//#define WD 	0x5744		
//#define RA 	0x5241	
//#define VER 0x564552
//#define WG 	0x5747	
//#define SIN 0x53494E
//#define SQR 0x535152
//#define TRI 0x545249
//#define STW 0x535457
//#define OFF 0x4F4646
#define SP 	0x5350
#define AC 	0x4143
#define FN 	0x464E
#define FF 	0x4646
#define S 	0x53
#define ST 	0x5354
#define MS 0X4D53
#define MICRO 0x4D494352
#define CS 	0x4353
#define EN 	0x454E
#define UN 	0x554E
//#define VR 	0x5652
#define HW 	0x4857
#define FSW 	0x465357
#define SW 	0x5357
#define STW 	0x535457
#define  AUT 0x415554
#define  YR 0x5952
#define  KD 0x4B44
#define  KI 0x4B49
#define  KP 0x4B50
#define  A 0x41
#define  H 0x48
#define  U 0x55
#define  US 0x5553


#define PORT_A 0x40020000U
#define PORT_B 0x40020400U
#define PORT_C 0x40020800U
#define PORT_D 0x40020C00U
#define PORT_E 0x40021000U
#define PORT_F 0x40021400U
#define PORT_G 0x40021800U
#define PORT_H 0x40021C00U
#define PORT_I 0x40022000U
#define PORT_J 0x40022400U
#define PORT_K 0x40022800U
#define PORT_K_END 0x40022BFFU

#define VERIFY_LIMITS(a, b, c, d)\
if(a / b < 1) c = a;\
else return d;

typedef struct command {
	unsigned short addr;
	uint32_t port_addr, addr3, freq;
	int signval;
	uint16_t pin_setting, pin_values;
	char instruction[4], signal[4], lenght, byte;
}command_c;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t memory_buffer[0x1000]={0};
unsigned char Rx_indx;
char adc_buffer[10];
uint8_t control_mode;
uint8_t enable;
uint8_t control;
float v_ref;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

GPIO_TypeDef* Discover_Port(command_c* command){
	if (command->port_addr >= PORT_A && command->port_addr < PORT_B) {__HAL_RCC_GPIOA_CLK_ENABLE(); return GPIOA;}
	else if (command->port_addr >= PORT_B && command->port_addr < PORT_C) {__HAL_RCC_GPIOB_CLK_ENABLE(); return GPIOB;}
	else if (command->port_addr >= PORT_C && command->port_addr < PORT_D) {__HAL_RCC_GPIOC_CLK_ENABLE(); return GPIOC;}
	else if (command->port_addr >= PORT_D && command->port_addr < PORT_E) {__HAL_RCC_GPIOD_CLK_ENABLE(); return GPIOD;}
	else if (command->port_addr >= PORT_E && command->port_addr < PORT_F) {__HAL_RCC_GPIOE_CLK_ENABLE(); return GPIOE;}
	else if (command->port_addr >= PORT_F && command->port_addr < PORT_G) {__HAL_RCC_GPIOF_CLK_ENABLE(); return GPIOF;}
	else if (command->port_addr >= PORT_G && command->port_addr < PORT_H) {__HAL_RCC_GPIOG_CLK_ENABLE(); return GPIOG;}
	else if (command->port_addr >= PORT_H && command->port_addr < PORT_I) {__HAL_RCC_GPIOH_CLK_ENABLE(); return GPIOH;}
	else if (command->port_addr >= PORT_I && command->port_addr < PORT_J) {__HAL_RCC_GPIOI_CLK_ENABLE(); return GPIOI;}
	else if (command->port_addr >= PORT_J && command->port_addr < PORT_K) {__HAL_RCC_GPIOJ_CLK_ENABLE(); return GPIOJ;}
	else if (command->port_addr >= PORT_K && command->port_addr < PORT_K_END) {__HAL_RCC_GPIOK_CLK_ENABLE(); return GPIOK;}
	return NULL;
}

int verify_instruction(char comm_instruction[4]) {
	switch (strlen(comm_instruction)) {
	case 1:
		return comm_instruction[0];
	case 2:
		return ((comm_instruction[0] << 8) + comm_instruction[1]);
	case 3:
		return ((comm_instruction[0] << 16) + (comm_instruction[1] << 8) + comm_instruction[2]);
	default:
		return 0;
	}
}



void execution(command_c *command){
	int instruction=0, signal=0, timeunit=0, counter=0;
	char buffer[128]={0};
//	uint16_t adcValue;
//	GPIO_InitTypeDef GPIO_Struct;
//	GPIO_TypeDef * port;
	instruction=verify_instruction(command->instruction);
	switch (instruction){
//		case MR:
//			print_message("Memory reading\n");
//			HAL_Delay(500);
//			for(int i=0; i<command->lenght; i++){
//				sprintf(buffer, " -> Position 0x%X: %X", (command->addr+i), memory_buffer[command->addr+i]);
//				print_message(buffer);
//				HAL_Delay(500);
//			}
//			break;
//		
//		case MW:
//			print_message("Memory Writing\n");
//			HAL_Delay(500);
//			for(int i=0; i<command->lenght; i++){
//				memory_buffer[command->addr+i]=command->byte;
//				sprintf(buffer, " -> Position 0x%X: %X", (command->addr+i), memory_buffer[command->addr+i]);
//				print_message(buffer);
//				HAL_Delay(500);
//			}
//			break;
//			
//		case MI:
//		case MO:
//			if (Discover_Port(command) != NULL) port=Discover_Port(command);
//			GPIO_Struct.Pin |= command->pin_setting;
//			GPIO_Struct.Pull = GPIO_NOPULL;
//			switch(instruction){
//				case MI:
//					print_message("Making Pin as Input\n");
//					GPIO_Struct.Mode=GPIO_MODE_INPUT;
//					break;
//				case MO:
//					print_message("Making Pin as Output\n");
//					HAL_GPIO_WritePin(port, command->pin_setting, GPIO_PIN_RESET);
//					GPIO_Struct.Mode=GPIO_MODE_OUTPUT_PP;
//					GPIO_Struct.Speed=GPIO_SPEED_FREQ_LOW;
//					break;
//			}
//			HAL_GPIO_Init(port,&GPIO_Struct);
//			HAL_Delay(500);
//			break;
//			
//		case RD:
//		case WD:
//			if (Discover_Port(command) != NULL) port=Discover_Port(command);
//			switch(instruction){
//				case RD:
//					print_message("Reading Pins\n");
//					for(int i=0; i<16; i++){
//						if(command->pin_setting & (1<<i)){
//							sprintf(buffer, "Pin %d Logic Value: %d",i, HAL_GPIO_ReadPin(port, (1<<i)));
//							print_message(buffer);
//					 }
//					}
//					break;
//				case WD:
//					print_message("Writing Pins\n");
//					for(int i=0; i<16; i++){
//						if(command->pin_values & (1<<i)){
//							HAL_GPIO_WritePin(port, (1<<i), GPIO_PIN_SET);
//						}
//						
//							else{
//								HAL_GPIO_WritePin(port, (1<<i), GPIO_PIN_RESET);
//							}
//								
//						
//					}	
//					break;
//			}
//			HAL_Delay(500);
//			break;

//		case RA:
//			HAL_ADC_Start(&hadc1);
//			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//			adcValue=HAL_ADC_GetValue(&hadc1);
//			HAL_ADC_Stop(&hadc1);
//			sprintf(buffer, "ADC Value : %hu", adcValue);
//			print_message(buffer);
//			HAL_Delay(500);
//			break;			
//			
//		case VER:	
//			print_message("v2.0 turno 2 grupo 8\n");
//			HAL_Delay(500);
//			break;
//		
////		case WG:
////			print_message("Wave generator\n");
////			signal = verify_instruction(command->signal);
////			switch(signal){
////				case SIN:
////					print_message("Wave sin\n");
////					wavegen_sin();
////					wavegen_freq_update(command->freq);
////					wavegen_start();
////					break;
////				case SQR:
////					wavegen_sqr();
////					wavegen_freq_update(command->freq);
////					wavegen_start();
////					break;
////				case TRI:
////					wavegen_tri();
////					wavegen_freq_update(command->freq);
////					wavegen_start();
////					break;
////				case STW:
////					wavegen_stw();
////					wavegen_freq_update(command->freq);
////					wavegen_start();
////					break;
////			}
////		break;
////			
////		case OFF:
////			wavegen_stop();
////			break;
////		
		case S:
			print_message("Starting. . .\r\n");
			MX_TIM6_Init();
			HAL_ADC_Start(&hadc1);
			HAL_TIM_Base_Start_IT(&htim6);
			break;
		case ST:
			HAL_TIM_Base_Stop_IT(&htim6);
			adc_signal->k_index = 0;
			print_message("Sampling Stopped.\r\n");
			break;
		case SP:
			timeunit = verify_instruction(adc_signal->timeunit);
			switch(timeunit){
				case MICRO:
					Tsample = adc_signal->units * 0.000001;
					break;
				case MS:
					Tsample = adc_signal->units * 0.001;
					break;
				case S:
					Tsample = adc_signal->units;
					break;
			}
			sprintf(buffer, "Ts = %f s\r\n", Tsample);
			print_message(buffer);
			break;
		case AC:
			adcChannel=Discover_AdcChannel(adc_signal->adc_channel);
			MX_ADC1_Init();
			sprintf(buffer, "Activating ADC Channel %d . . .\r\n", adc_signal->adc_channel);
			print_message(buffer);
			break;
//		case FN:
//			filter = 1;
//			print_message("Filter ON\r\n");
//			break;
//		case FF:
//			filter = 0;
//			print_message("Filter OFF\r\n");
//			break;
		
		case CS:
			signal=verify_instruction(command->signal);
			switch(signal){
				case 0x30:
					control_mode=0;
					break;
				case 0x31:
					control_mode=1;
					break;
			}
			break;
			
			case EN:
			signal=verify_instruction(command->signal);
			switch(signal){
				case 0x30:
					control=0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //PINO ENABLE PA8
					break;
				case 0x31:
					control=1;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //PINO ENABLE PA8
					break;
			}
			break;
			
			case UN:
				if(!control_mode && control){
					if (command->signval >=1 && command->signval<=100){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //pino fw pa9
					}
					else if (command->signval <=-1 && command->signval>=-100){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); ////pino rv pa9
					}
					counter=abs(command->signval);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(counter*540));
				}
			break;
			
//			case VR:
//				if(control_mode && control){
//					v_ref = command->signval;
//					if (v_ref >=1 && v_ref<=160){
//						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //pino fw pa9
//					}
//					else if (v_ref <=-1 && v_ref<=-160){
//						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); ////pino rv pa9
//					}
////					atualiza_PWM();
//				}
//			break;	
				
			case HW:
			timeunit = verify_instruction(sampling->timeunit);
			switch(timeunit){
				case MICRO:
					Tsample2 = sampling->units * 0.000001;
					break;
				case MS:
					Tsample2 = sampling->units * 0.001;
					break;
				case S:
					Tsample2 = sampling->units;
					break;
			}
			sprintf(buffer, "Ts = %f s\r\n", Tsample2);
			print_message(buffer);
			break;
			
			case FSW:
				sampling->multiplier = command->signval;			//multiplier predefinido com 1
				sprintf(buffer, "Multiplier = %d \r\n", sampling->multiplier);
				print_message(buffer);
			break;
			
			case SW:
				sprintf(buffer, "Sampling. . . \r\n");
				print_message(buffer);
		
				MX_TIM4_Init();
				MX_TIM7_Init();
				HAL_TIM_Base_Start_IT(&htim7);
				HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1); 
			break;
			
			case STW:
				HAL_TIM_Base_Stop_IT(&htim7);
				HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
				sampling->k_index = 0;
				sprintf(buffer, "Sampling Stopped. \r\n");
				print_message(buffer);
			break;
			
			case H:
			timeunit = verify_instruction(pid->timeunit);
			switch(timeunit){
				case MICRO:
					Tsample3 = pid->units * 0.000001;
					break;
				case MS:
					Tsample3 = pid->units * 0.001;
					break;
				case S:
					Tsample3 = pid->units;
					break;
			}
			sprintf(buffer, "Ts = %f s\r\n", Tsample3);
			print_message(buffer);
			break;
			
			case AUT:
				pid->aut = command->signval;
			break;
			
//			case YR:
//				pid->yr = command->signval;
//			break;
//			
//			case KD:
//				pid->kd = command->signval;
//				pid->kd_h = pid->kd/Tsample3;
//			break;
//			
//			case KI:
//				pid->ki = command->signval;
//				pid->ki_h = pid->ki * Tsample3;
//			break;
//			
//			case KP:
//				pid->kp = command->signval;
//				pid->kp_h = pid->kp;
//			break;
//			
//			case A:
//				pid->a = command->signval;
//			break;
			
			case U:
				print_message("Starting. . .\r\n");
				MX_TIM9_Init();
				HAL_TIM_Base_Start_IT(&htim9);
				pid->sum_e = 0;
			break;
			
			case US:
				HAL_TIM_Base_Stop_IT(&htim9);
				pid->index = 0;
				pid->p_index = 0;
//				pid->sum_e = 0;
//				pid->sum_e_bkp = 0;
//				pid->e_ant = 0;
				print_message("Sampling Stopped.\r\n");
			break;
				
			
		default:
			break;
	}
}

int parsing(char buffer[128], command_c *command) {
	unsigned int i=0, j=0;
	char lixo[128], *p=NULL, buffer_aux[128]={""}; 
	
	strcpy(buffer_aux, buffer);
	p = strtok(buffer, " ");
	if (p != NULL) strcpy(command->instruction, p);
	strcpy(buffer, buffer_aux);
	
	switch (verify_instruction(command->instruction)){
//		case MR:
//			if(sscanf(buffer, "%s %x %x %s", command->instruction, &i, &j, lixo) > 3 || sscanf(buffer, "%s %x %x %s", command->instruction, &i, &j, lixo) <3)return 1;
//			VERIFY_LIMITS(i, 0x10000, command->addr, 2)
//			VERIFY_LIMITS(j, 0x100, command->lenght, 3)
//			break;
//		
//		case MW:
//			if(sscanf(buffer, "%s %x %x %x %s", command->instruction, &i, &j, &k, lixo) > 4 || sscanf(buffer, "%s %x %x %x %s", command->instruction, &i, &j, &k, lixo) < 4) return 1;
//			VERIFY_LIMITS(i, 0x10000, command->addr, 2)
//			VERIFY_LIMITS(j, 0x100, command->lenght, 3)
//			VERIFY_LIMITS(k, 0x100, command->byte, 4)
//			break;
//		
//		case MI:
//		case MO:
//			if(sscanf(buffer, "%s %x %x %s", command->instruction, &i, &j, lixo) > 3 || sscanf(buffer, "%s %x %x %s", command->instruction, &i, &j, lixo) < 3) return 1;
//			VERIFY_LIMITS(i, 0x100000000, command->port_addr, 5)
//			VERIFY_LIMITS(j, 0x10000, command->pin_setting, 6)
//			break;
//		
//		case RD:
//			if(sscanf(buffer, "%s %x %x %s", command->instruction, &i, &j, lixo) > 3 || sscanf(buffer, "%s %x %x %s", command->instruction, &i, &j, lixo) < 3) return 1;
//			VERIFY_LIMITS(i, 0x100000000, command->port_addr, 5)
//			VERIFY_LIMITS(j, 0x10000, command->pin_setting, 6)
//			break;
//		
//		case WD:
//			if(sscanf(buffer, "%s %x %x %x %s", command->instruction, &i, &j, &k, lixo) > 4 || sscanf(buffer, "%s %x %x %x %s", command->instruction, &i, &j, &k, lixo) < 4) return 1;
//			VERIFY_LIMITS(i, 0x100000000, command->port_addr, 5)
//			VERIFY_LIMITS(j, 0x10000, command->pin_setting, 6)
//			VERIFY_LIMITS(k, 0x10000, command->pin_values, 7)
//			break;
//		
//		case RA:
//			if(sscanf(buffer, "%s %x %s", command->instruction, &i, lixo) > 2 || sscanf(buffer, "%s %x %s", command->instruction, &i, lixo) < 2) return 1;
//			VERIFY_LIMITS(i, 0x12, command->addr3, 8)
//			break;
//		
//		case VER:
//			if(sscanf(buffer, "%s %s", command->instruction, lixo) > 1 || sscanf(buffer, "%s %s", command->instruction, lixo) < 1) return 1;
//			break;
		
		case 0x3F: 
			if (sscanf(buffer, "%s %s", command->instruction, lixo) > 1 || sscanf(buffer, "%s %s", command->instruction, lixo) < 1) return 1;
			break;
		
//		case WG:
//			if(sscanf(buffer, "%s %s %x %s", command->instruction, command->signal, &j, lixo) > 3 || sscanf(buffer, "%s %s %x %s", command->instruction, command->signal, &j, lixo) < 3) return 1;
//			VERIFY_LIMITS(j, 0x150, command->freq, 8)
//			break;
//			
//		case OFF:
//			if(sscanf(buffer, "%s %s", command->instruction, lixo) > 1 || sscanf(buffer, "%s %s", command->instruction, lixo) < 1) return 1;
//			break;
		
		case SP:
			if(sscanf(buffer, "%s %s %d %s", command->instruction, adc_signal->timeunit, &i, lixo) > 3 || sscanf(buffer, "%s %s %d %s", command->instruction, adc_signal->timeunit, &i, lixo) < 3) return 1;
			VERIFY_LIMITS(i, 0x10000, adc_signal->units, 8)
			break;
			
		case AC:
			if(sscanf(buffer, "%s %x %s", command->instruction, &i, lixo) > 2 || sscanf(buffer, "%s %x %s", command->instruction, &i, lixo) < 2) return 1;
			VERIFY_LIMITS(i, 0x10, adc_signal->adc_channel, 8)
			break;
			
		case FF:
			if(sscanf(buffer, "%s %s", command->instruction, lixo) > 1 || sscanf(buffer, "%s %s", command->instruction, lixo) < 1) return 1;
			break;
		
		case FN:
			if(sscanf(buffer, "%s %s", command->instruction, lixo) > 1 || sscanf(buffer, "%s %s", command->instruction, lixo) < 1) return 1;
			break;
		
		case S:
			if(sscanf(buffer, "%s %d %s", command->instruction, &i, lixo) > 2) return 1;
			VERIFY_LIMITS(i, 0x10000, adc_signal->k, 8)
			break;
		
		case ST:
			if(sscanf(buffer, "%s %s", command->instruction, lixo) > 1 || sscanf(buffer, "%s %s", command->instruction, lixo) < 1) return 1;
			break;
		
		case CS:
			if(sscanf(buffer, "%s %s %s", command->instruction, command->signal, lixo) > 2 || sscanf(buffer, "%s %s %x %s", command->instruction, command->signal, &j, lixo) < 2) return 1;
			break;
			
		case EN:
			if(sscanf(buffer, "%s %s %s", command->instruction, command->signal, lixo) > 2 || sscanf(buffer, "%s %s %x %s", command->instruction, command->signal, &j, lixo) < 2) return 1;
			break;
			
		case UN:
			if(sscanf(buffer, "%s %d %s", command->instruction, &i, lixo) > 2 || sscanf(buffer, "%s %d %s", command->instruction, &i, lixo) < 2) return 1;
			VERIFY_LIMITS(i, 101, command->signval, 8)
			break;
			
//		case VR:
//			if(sscanf(buffer, "%s %d %s", command->instruction, &i, lixo) > 2 || sscanf(buffer, "%s %d %s", command->instruction, &i, lixo) < 2) return 1;
//			VERIFY_LIMITS(i, 0xA0, command->signval, 8)
//			break;	

		case HW:
			if(sscanf(buffer, "%s %s %d %s", command->instruction, sampling->timeunit, &i, lixo) > 3 || sscanf(buffer, "%s %s %d %s", command->instruction, sampling->timeunit, &i, lixo) < 3) return 1;
			VERIFY_LIMITS(i, 0x10000, sampling->units, 8)
			break;	

		case FSW:
			if (sscanf(buffer, "%s %d %s", command->instruction, &command->signval, lixo) > 2 || sscanf(buffer, "%s %d %s", command->instruction, &command->signval, lixo) < 2) return 1;
			if (command->signval > 100 || command->signval < -100) return 1;
			break;

		case SW:
			if(sscanf(buffer, "%s %d %s", command->instruction, &i, lixo) > 2) return 1;
			VERIFY_LIMITS(i, 0x10000, sampling->k, 8)
			break;

		case STW:
			if(sscanf(buffer, "%s %s", command->instruction, lixo) > 1 || sscanf(buffer, "%s %s", command->instruction, lixo) < 1) return 1;
			break;		
		
		case AUT:
			if(sscanf(buffer, "%s %d %s", command->instruction, &command->signval, lixo) > 2 || sscanf(buffer, "%s %s", command->instruction, lixo) < 2) return 1;
			break;
		
		case YR:
			if(sscanf(buffer, "%s %d %s", command->instruction, &command->signval, lixo) > 2 || sscanf(buffer, "%s %s", command->instruction, lixo) < 2) return 1;
			break;	
			
		case KD:
			if(sscanf(buffer, "%s %d %s", command->instruction, &command->signval, lixo) > 2 || sscanf(buffer, "%s %s", command->instruction, lixo) < 2) return 1;
			break;
		
		case KI:
			if(sscanf(buffer, "%s %d %s", command->instruction, &command->signval, lixo) > 2 || sscanf(buffer, "%s %s", command->instruction, lixo) < 2) return 1;
			break;
			
		case KP:
			if(sscanf(buffer, "%s %d %s", command->instruction, &command->signval, lixo) > 2 || sscanf(buffer, "%s %s", command->instruction, lixo) < 2) return 1;
			break;
			
		case A:
			if(sscanf(buffer, "%s %d %s", command->instruction, &command->signval, lixo) > 2 || sscanf(buffer, "%s %s", command->instruction, lixo) < 2) return 1;
			break;
			
		case H:
			if(sscanf(buffer, "%s %s %d %s", command->instruction, pid->timeunit, &i, lixo) > 3 || sscanf(buffer, "%s %s %d %s", command->instruction, pid->timeunit, &i, lixo) < 3) return 1;
			VERIFY_LIMITS(i, 0x10000, pid->units, 8)
			break;
			
		case U:
			if(sscanf(buffer, "%s %s", command->instruction, lixo) > 1 || sscanf(buffer, "%s %s", command->instruction, lixo) < 1) return 1;
			break;
		
		case US:
			if(sscanf(buffer, "%s %s", command->instruction, lixo) > 1 || sscanf(buffer, "%s %s", command->instruction, lixo) < 1) return 1;
			break;
			
		default:
			return 1;
	}
	return 0;
}



char push_FIFO(char buffer[128])
{
	for (int i=0;i<(255);i++)
		if(Rx_Buffer[i]=='\0'){
			return buffer[Rx_indx];
		}
		else buffer[Rx_indx++]=Rx_Buffer[i];
		return buffer[Rx_indx];
 
}

char empty_FIFO(char Rx_Buffer[128])
{
	int i;
	
	for (i=0;i<(255);i++) 
				Rx_Buffer[i]=0;
	
	return Rx_Buffer[i];
}

	char BackSpace(char buffer[128])
{
	if(Rx_indx != 0) 
	{
		buffer[--Rx_indx]=0;
	}

	if(Rx_indx == 0)
	{
		buffer[1]=0;
		buffer[0]=0;
	}
	return buffer[Rx_indx];
}

char empty_buffer(char buffer [128])
{
	uint8_t i;
	
	for (i=0;i<255;i++) 
				buffer[i]=0;
				
	return buffer[i];
}

//void empty_x_buffer(signal_s *adc_signal)
//{
//	for(int i=0; i<MAX_INDEX; i++)
//		adc_signal->x[i]=0;
//}


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
	
	char buffer[128];
	int i = 0, valid=0;
	command_c *command;
	char last_succ_command [128];
	uint32_t k = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
	command = malloc(sizeof(command_c));
	adc_signal = malloc(sizeof(signal_s));
	adc_signal->index = 1;
	adc_signal->k_index = 0;
	sampling = malloc(sizeof(sampling_s));
	sampling->multiplier = 1;
	sampling->index = 1;
	sampling->k_index = 0;
	pid->sum_e = 0;
	pid->e_ant=0;
	pid->yr = 1;
	pid->kd = 0;
	pid->ki = 0;
	pid->kp = 0;
	pid->kd_h = 3.35;
	pid->ki_h = 0;
	pid->kp_h = 0;
	pid->a = 0.33;
//	pid->index=0;
//	pid->p_index=0;
	pid->u=0;
	pid->y = 0;
	pid->sum_e_bkp = 0;
	pid->aut = 1;
	pid->U_sat_a = 1.0;
	pid->U_sat_b = -1.0;
	pid->p_index=0;
	init_UART3();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	empty_x_buffer(adc_signal);
//	wavegen_init();
	print_message(">");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		switch(pid_print){
//			case 1:
//				sprintf(sampling_buffer, "%d %.2f %.2f %.2f", pid->p_index, x[pid->index], pid->u, pid->sum_e);		
//				print_message(sampling_buffer);
//				pid_print = 0;
//				pid->index++;
//				pid->p_index++;
//				if(pid->index == 20){
//					pid->index=0;
//				}
//			break;
//		}
		switch(sampling_print){
			case 1:
				sprintf(sampling_buffer, "N:%d V:%d", sampling->k_index, sampling->x[sampling->index]);		//pode ser index-1
			if(sampling->k != 0)
				k = sampling->k - sampling->k_index;
			else k = 0;
				switch(k){
					case 1:
						HAL_TIM_Base_Stop_IT(&htim7);
						HAL_TIM_Base_Stop_IT(&htim4);
						sampling->k_index = 0;
//						HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
						break;
					default:
						sampling->k_index++;
			}
			print_message(sampling_buffer);
			sampling_print = 0;
		}
		switch (adc_print){
			case 1:
				sprintf(adc_buffer, "%d %d", adc_signal->k_index, send);
			if(adc_signal->k != 0)
				k = adc_signal->k - adc_signal->k_index;
			else k = 0;
				switch(k){
					case 1:
						HAL_TIM_Base_Stop_IT(&htim6);
						adc_signal->k_index = 0;
						break;
					default:
						HAL_ADC_Start(&hadc1);
						adc_signal->k_index++;
			}
			print_message(adc_buffer);
			adc_print = 0;
		}
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		newMessage();
		if(receive_flag){
			switch(Rx_Buffer[0]){
				case 8:
					BackSpace(buffer);
					printf(" %s \n", Rx_Buffer);
					printf(" %s \n", buffer);
					break;
//				case '.':
//					printf(" %s \n", buffer);
//					break;
				case '$':
					empty_buffer(buffer);
					Rx_indx = 0;
					parsing(last_succ_command,command);
					execution(command);
					break;
				case 27:
					empty_buffer(buffer);
					Rx_indx = 0;
					print_message("Buffer has been cleared\n");
					break;
				case 0x3F:
					print_commands();
					HAL_Delay(500);
					break;
				case 0x2F:
					if((command->signval+5) < 100) command->signval+=5;
					else command->signval=100;
					if (command->signval <0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
					else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(command->signval*540));
					break;
				case 0x5C:
					if((command->signval-5) > -100) command->signval-=5;
					else command->signval=-100;
					if (command->signval <0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
					else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(command->signval*540));
					break;
				default:
					push_FIFO(buffer);
					break;
				}
			
			
			while (buffer[i]){
				buffer[i] = toupper(buffer[i]);
				i++;
			}
			i=0;
			valid= parsing(buffer, command);
			switch(valid){
				case 0:
					execution(command);
					strcpy(last_succ_command,(char*)buffer);
					print_message("\nSuccess\n");
					empty_buffer(buffer);
					Rx_indx = 0;
					break;
				default:
					print_error(valid);
			}
				

			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			receive_flag = 0;
			print_message(">");
		}
		HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
