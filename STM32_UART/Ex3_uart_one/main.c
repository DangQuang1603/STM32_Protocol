/*
	*File: main.c
	*Author: Le Dang Quang
	*Date: 7/10/2023
	*Disciption: This file is create to transmitt data by UART communication
*/
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define Tx GPIO_Pin_6
#define Rx GPIO_Pin_7

void uartInit();
void uartDelay();
void uartEnd();
void transmitData(uint8_t data);

	
int main(){
	uartInit();
	uint8_t data[10] = {11,22,33,44,55,66,77,88,99,10};
	uint8_t i = 0;
	
	while(1){
		for(int i = 0; i < 10000000; i++){};
		transmitData(data[i]);
		i++;
	}
}

//Declare uart port

void uartInit(){
	GPIO_InitTypeDef uartInit;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	uartInit.GPIO_Pin = Tx;
	uartInit.GPIO_Mode = GPIO_Mode_Out_PP;
	uartInit.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &uartInit);
	
	uartInit.GPIO_Pin = Rx;
	uartInit.GPIO_Mode =  GPIO_Mode_IPU;
	uartInit.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &uartInit);
	
	GPIO_SetBits(GPIOA, Tx);
}

// set bit time
void uartDelay(){
	for(int i = 0; i < 100000; i++){};
}

// set end bit 
void uartEnd(){
	GPIO_SetBits(GPIOA, Tx);
	uartDelay();
	uartDelay();
}

// transmit data
void transmitData(uint8_t data){
	GPIO_ResetBits(GPIOA, Tx);
	uint8_t checkParityBits = 0;
	uartDelay();
	for(uint8_t i = 0; i < 8; i++){
		if(data & (0x80 >> i)){
			checkParityBits++;
			GPIO_SetBits(GPIOA, Tx);
		}else GPIO_ResetBits(GPIOA, Tx);
		uartDelay();
	}
	if(checkParityBits % 2 != 0) GPIO_SetBits(GPIOA, Tx);
	else GPIO_ResetBits(GPIOA, Tx);
	uartDelay();
	uartEnd();
}
