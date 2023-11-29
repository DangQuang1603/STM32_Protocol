/*
	* File: main.c
	* Author: Le Quang
	* Date: 25/09/2023
	* Description: transmit data from master to slave via SPI by software
*/
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

// Define SPI pin
#define CLK GPIO_Pin_10
#define MOSI GPIO_Pin_11
#define SS GPIO_Pin_12

void config();
void startslv1();
void endslv1();
void clock();
void transmitdata(uint8_t data);

int main(){
	config();
	uint8_t data = 7;
	while(1){
		transmitdata(data);
	}
}

// Configurate SPI pin
void config(){
	GPIO_InitTypeDef masterport;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	masterport.GPIO_Pin = CLK|MOSI|SS;
	masterport.GPIO_Mode = GPIO_Mode_Out_PP;
	masterport.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &masterport);
}

// Generate clock using for loop
void clock(){
	GPIO_SetBits(GPIOA, CLK);
	for(int i  = 0; i < 50000000; i++){};
	GPIO_ResetBits(GPIOA, CLK);
	for(int i  = 0; i < 50000000; i++){};
}

// Send end signal to slave
void endslv1(){
	GPIO_SetBits(GPIOA, SS);
	for(int i = 0; i < 100000000; i++){};
}

// Send start signal to slave
void startslv1(){
	GPIO_ResetBits(GPIOA, SS);
}

// Function transmit data
void transmitdata(uint8_t data){
	// Send start signal
	startslv1();
	// Transmit each bit to slave
	for(uint8_t i  = 0; i < 8; i++){
		if((data & (0x80 >> i)) == 0 ) GPIO_ResetBits(GPIOA, MOSI);
		else GPIO_SetBits(GPIOA, MOSI);
		clock();
	}
	// Send end signal
	endslv1();
	
}
		
