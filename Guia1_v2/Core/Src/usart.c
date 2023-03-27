/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "string.h"

uint8_t UART3Rx_Buffer[128];
uint8_t Rx_Buffer[128];
int receive_flag=0;
volatile uint8_t UART3Rx_index=0;
uint8_t Tx_Buffer[256];
int transmit_flag=0;
volatile uint8_t Tx_index=0;


/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void init_UART3(){
	HAL_UART_Receive_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index] , 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if(huart->Instance == USART3){
		if (UART3Rx_Buffer[UART3Rx_index]=='\n'){
			UART3Rx_Buffer[UART3Rx_index]='\0';
		}
		else{
		UART3Rx_index++;
		UART3Rx_index &= ~(1<<7);
		}
		HAL_UART_Receive_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
	} 
}

void newMessage(){
	static int local_index=0;
	int out_index=0;
	while(local_index != UART3Rx_index){
		Rx_Buffer[out_index]=UART3Rx_Buffer[local_index];
		out_index++;
		local_index++;
		local_index &= ~(1<<7);
		receive_flag=1;
	}
		Rx_Buffer[out_index] = '\0';
		
}

int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 100);
	return ch;
}

void print_message(char* message){
	strcpy((char*)Tx_Buffer, message);
	HAL_UART_Transmit_IT(&huart3, &Tx_Buffer[Tx_index], 1);
	while(transmit_flag==0);
	transmit_flag=0;
}

void print_commands(){
	char buffer[500]={""};
	sprintf(buffer, "->Memory Read: MR <addr> <length>\n\r Le e envia para o computador um segmento de memoria (que pode ser so um byte)\n\r");
	print_message(buffer);
	sprintf(buffer, "->Memory Write: MW <addr> <length> <byte>\n\r Escreve um valor num segmento de memoria (que pode ser so um byte)\n\r");
	print_message(buffer);
	sprintf(buffer, "->Make Pin Input: MI <port addr> <pin setting>\n\r Programa pinos de uma porta como input\n\r");
	print_message(buffer);
	sprintf(buffer, "->Make Pin Output: MO <port addr> <pin setting>\n\r Programa pinos de uma porta como output\n\r");
	print_message(buffer);
	sprintf(buffer, "->Read Dig Input: RD <port addr> <pin setting>\n\r Le e envia para o computador o valor dos bits especificados de uma porta\n\r");
	print_message(buffer);
	sprintf(buffer, "->Read Dig Output: WD <port addr> <pin setting> <pin values>\n\r Escreve um valor de (ate) 8 bits numa porta de output\n\r");
	print_message(buffer);
	sprintf(buffer, "->Analog Read: RA <addr3>\n\r Inicia a conversao, le o valor resultante e envia para o computador como valor inteiro\n\r");
	print_message(buffer);
	sprintf(buffer, "-><BCKSP>:\n\r Backspace - Limpa o ultimo caracter recebido\n\r");
	print_message(buffer);
	sprintf(buffer, "-><ESC>:\n\r Abort - Limpa todos os caracteres recebidos\n\r");
	print_message(buffer);
	sprintf(buffer, "->$:\n\r Limpa todos os caracteres recebidos e repete o ultimo comando\n\r");
	print_message(buffer);
	sprintf(buffer, "->?\n\r Help - Fornece uma lista dos comandos validos\n\r");
	print_message(buffer);
	sprintf(buffer, "->VER\n\r Version - Devolve uma string que identifica a versao de firmware, bem como o turno e o grupo de trabalho\n\r");
	print_message(buffer);
}

void print_error(int valid){
	switch(valid){
		case 1:
			print_message("Command doesn't exist!\n\r");
			break;
		case 2:
			print_message("Invalid memory address!\n\r");
			break;
		case 3:
			print_message("Invalid length!\n\r");
			break;
		case 4:
			print_message("Invalid byte!\n\r");
			break;
		case 5:
			print_message("Invalid port address!\n\r");
			break;
		case 6:
			print_message("Invalid pin setting!\n\r");
			break;
		case 7:
			print_message("Invalid pin values!\n\r");
			break;
		case 8:
			print_message("Invalid address!\n\r");
			break;
		
		default:
			break;
	}
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART3){
		Tx_index++;
		Tx_index &=~(1<<7);
		if (Tx_Buffer[Tx_index]!='\0')
			HAL_UART_Transmit_IT(&huart3, &Tx_Buffer[Tx_index], 1);
		else{
			Tx_Buffer[Tx_index++]='\r';
			Tx_Buffer[Tx_index]='\n';
			HAL_UART_Transmit(&huart3, &Tx_Buffer[--Tx_index], 2, 100);
			transmit_flag=1;
			Tx_index=0;
		}
	}
}




/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
