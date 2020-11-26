#include "fona.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

UART_HandleTypeDef huart1;

#define TIMEOUT 50

#define BUFFER_SIZE 32

void sendAT(const char * string){
	uint8_t buffer[BUFFER_SIZE];
	
	snprintf((char*)buffer, BUFFER_SIZE, "%s\r\n", string);
	HAL_UART_Transmit(&huart1, buffer, strlen((const char*)buffer), TIMEOUT);
	
	HAL_StatusTypeDef ret = HAL_OK;
	while(ret == HAL_OK){
		ret = HAL_UART_Receive(&huart1, buffer, 1, TIMEOUT);
	}
}

void fona_init(void){
	huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);
}

void fona_voiceVolume(uint8_t vol){
	if(vol > 100) return;
	char buffer[BUFFER_SIZE];
	snprintf(buffer,BUFFER_SIZE,"AT+CLVL=%u",vol);
	sendAT(buffer);
}

void fona_ringVolume(uint8_t vol){
	if(vol > 100) return;
	char buffer[BUFFER_SIZE];
	snprintf(buffer,BUFFER_SIZE,"AT+CRSL=%u",vol);
	sendAT(buffer);
}

void fona_call(const char* number){
	char buffer[BUFFER_SIZE];
	snprintf(buffer,BUFFER_SIZE,"ATD%s;",number);
	sendAT(buffer);
}

void fona_hangUp(void){
	sendAT("ATH");
}

void fona_pickUp(void){
	sendAT("ATA");
}

void fona_sms(const char* number, const char* msg){
	char buffer[BUFFER_SIZE];
	
	sendAT("AT+CMGF=1");
	
	snprintf(buffer,BUFFER_SIZE,"AT+CMGS=\"%s\"",number);
	sendAT(buffer);
	sendAT(msg);
	
	buffer[0] = 0x1A; // STRG + Z
	buffer[1] = 0x00;
	sendAT(buffer);
}


