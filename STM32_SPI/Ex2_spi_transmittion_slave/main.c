/*
	* File: main.c
	* Author: Le Quang
	* Date: 25/09/2023
	* Description: recive data from master via SPI by software
*/
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

//Define SPI pin
#define CLK GPIO_Pin_10 // Clock pin
#define MISO GPIO_Pin_11 // Data pin
#define SS GPIO_Pin_12	// Enable slave pin

void config();
void activeSlv();
void recieveData(uint8_t data);

int main(){
	config();
	uint8_t data = 0;
	while(1){
		recieveData(data);
	}
}

// Configurate SPI pin
void config(){
	GPIO_InitTypeDef slaveport;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	slaveport.GPIO_Pin = CLK|MISO|SS;
	slaveport.GPIO_Mode = GPIO_Mode_IPD;
	slaveport.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &slaveport);
	
	slaveport.GPIO_Pin = SS;
	slaveport.GPIO_Mode = GPIO_Mode_IPU;
	slaveport.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &slaveport);
}

// Wait for start signal from master
void activeSlv(){
	while(GPIO_ReadInputDataBit(GPIOA, SS) == 1);
}

// Recieve data from master
void recieveData(uint8_t data){
	// Wait for start signal
	activeSlv();
	data = 0;
	// Recieve each byte
	for(uint8_t i = 0; i < 8; i++){
		while(GPIO_ReadInputDataBit(GPIOA,CLK)== 0);
		if(GPIO_ReadInputDataBit(GPIOA, MISO ) == 1){
			data = data | (0x80>>i); // save byte recieve
		}
		while(GPIO_ReadInputDataBit(GPIOA,CLK)== 1);
	}
}
