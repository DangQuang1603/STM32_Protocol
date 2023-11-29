
/*
	*File: main.c
	*Author: Le Dang Quang
	*Date: 7/10/2023
	*Disciption: This file is create to recieve data by UART communication
*/
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define TX GPIO_Pin_6
#define RX GPIO_Pin_7
#define PORT GPIOB

void uartInit();
void uartRecDelay();
void waitStart();
void waitEnd();
uint16_t recieveData();
void arrRecieve(uint16_t arrRecieve[], uint8_t size);

uint16_t data = 0;
uint16_t dataArr[10] = {0};


int main(){
	uartInit();
		arrRecieve(dataArr, 10);
}

// Initial uart port
void uartInit(){
	GPIO_InitTypeDef uartPort;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
	
	uartPort.GPIO_Pin = TX;
	uartPort.GPIO_Mode = GPIO_Mode_Out_PP;
	uartPort.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(PORT, &uartPort);
	
	uartPort.GPIO_Pin = RX;
	uartPort.GPIO_Mode = GPIO_Mode_IPD;
	uartPort.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(PORT, &uartPort);
	
	GPIO_SetBits(PORT, TX);
}

void uartRecDelay(){
 	for(int i = 0; i < 100000; i++){};
}

void waitStart(){
	while(GPIO_ReadInputDataBit(PORT, RX) == 1);
	for(int i = 0; i < 150000; i++){};
}

void waitEnd(){
	while(GPIO_ReadInputDataBit(PORT, RX) == 0);
	for(int i = 0; i < 150000; i++){};
}

uint16_t recieveData(){
	uint16_t data = 0;
	uint8_t checkParityBits = 0;
	waitStart();
	for(uint8_t i = 0; i < 9; i++){
			if(GPIO_ReadInputDataBit(PORT, RX) == 1) {
				data = data | (0x100 >> i);
				checkParityBits++;
			}
			uartRecDelay();
	}
	waitEnd();
	if(checkParityBits % 2 != 0) data =0;
	else data =  data >> 1;
	return data;
}

void arrRecieve(uint16_t arrRecieve[], uint8_t size){
	for(uint8_t i = 0; i < size; i++){
		dataArr[i] = recieveData();
	}
}
