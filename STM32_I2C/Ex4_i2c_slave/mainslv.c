/*
*File: main.c
*Author: Le Dang Quang
*Date: 14/10/2023
*Disciption: This file is create to recive/send data by i2c communication to master 
*/

/*--------------------------------CODE----------------------------------------*/

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

#define SLAVE_ADDRESS 0x58
#define SDA GPIO_Pin_6
#define SCL GPIO_Pin_7
#define PORT GPIOB
#define ACK 0
#define NACK 1

uint32_t millisecond = 0;
uint8_t data[10] = {0};
uint8_t address = 0;

void timer2Init();
void TIM2_IRQHandler();
void delayMs(uint32_t ms);
void configInputSda();
void configOutputSda();
void configInit();
void waitstartI2c();
void waitendI2c();
uint8_t recieveAddress();
uint8_t recieveData();
void slaveI2cRecieve(uint8_t data[], uint8_t size);

int main(){
	timer2Init();
	configInit();
	while (1){
	slaveI2cRecieve(data,10);
	}
}

/********************************************************************************
*														Config Function
**********************************************************************************/

/*
* Function: timer2Init
* Description: initial and active timer2 and relevant interrupt
* Input:
*   None
* Ouput: 
*   None
*/
void timer2Init(){
	TIM_TimeBaseInitTypeDef timer;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	timer.TIM_Prescaler = 3600;
	timer.TIM_Period = 19;
	timer.TIM_ClockDivision = TIM_CKD_DIV1;
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&timer);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
	
	NVIC_InitTypeDef initIT;
	initIT.NVIC_IRQChannel = TIM2_IRQn;
	initIT.NVIC_IRQChannelPreemptionPriority = 0x00;
	initIT.NVIC_IRQChannelSubPriority = 0x00;
	initIT.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&initIT);
}

/*
* Function: configInputSda
* Description: configure port A8 as input
* Input:
*  	None
* Ouput: 
*   None
*/void configInputSda(){
	GPIOB->CRL &= ~(0xF<<24);
	GPIOB->CRH |= 1<<27;
	GPIOB->ODR |= 1<<6;
}

/*
* Function: configInputSda
* Description: configure port A8 as output
* Input:
*   None
* Ouput: 
*   None
*/
void configOutputSda(){
	GPIOB->CRL &= ~(0xF << 24);
	GPIOB->CRL |= 1<<25;
	GPIOB->ODR |= 1<<6;
}

/*
* Function: configInit
* Description: initial configure available port
* Input:
*   None
* Ouput: 
*   None
*/
void configInit(){
	GPIO_InitTypeDef initIcPort;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	//initIcPort.GPIO_Pin = SDA;
	//initIcPort.GPIO_Mode = GPIO_Mode_Out_PP;
	//initIcPort.GPIO_Speed = GPIO_Speed_2MHz;
	
	initIcPort.GPIO_Pin = SCL|SDA;
	initIcPort.GPIO_Mode = GPIO_Mode_IPU;
	initIcPort.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &initIcPort);
	
	//GPIO_SetBits(GPIOB, SDA);
}

/***********************************************************************
														Timer interrupt Function
************************************************************************/

/*
* Function: TIM2_IRQHandler
* Description: this finction increase millisecond variable by 1 when interrupt
* Input:
*   None
* Ouput: 
*   None
*/
void TIM2_IRQHandler(){
	millisecond++;
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

/*
* Function: delayMs
* Description: dalay time 
* Input:
*   ms: time want to delay by millisecond
* Ouput: 
*   None
*/
void delayMs(uint32_t ms){
	millisecond = 0;
	while(millisecond < ms);
}

/***********************************************************************
														Implement Function
************************************************************************/

/*
* Function: clock
* Description: create clock pulse for can communicate transmittion
* Input:
*   None
* Ouput: 
*   None
*/
void clock(){
	GPIO_SetBits(GPIOA,SCL);
	delayMs(10);
	GPIO_ResetBits(GPIOA, SCL);
	delayMs(10);
}

/*
* Function: waitstartI2c
* Description: wait fot SDA switch  line from High to Low voltage before SCL goes like this
* Input:
*   None
* Ouput: 
*   None
*/
void waitstartI2c(){
	configInputSda();
	while (GPIO_ReadInputDataBit(PORT, SDA) == 1);
	while (GPIO_ReadInputDataBit(PORT, SCL) == 1);
	delayMs(10);
}

/*
* Function: startI2c
* Description: wait for SDA switch line from High to Low voltage before SCL goes like this to generating Stop signal
* Input:
*   None
* Ouput: 
*   None
*/
void waitendI2c(){
	while (GPIO_ReadInputDataBit(PORT, SCL) == 0);
	while (GPIO_ReadInputDataBit(PORT, SDA) == 0);
}

/*
* Function: recieveAddres
* Description: recieve address fram from master
* Input:
*   None
* Ouput: 
*   None
*/
uint8_t recieveAddress(){
	uint8_t data = 0;
	for(uint8_t i = 0; i < 8; i++){
		while (GPIO_ReadInputDataBit(PORT, SCL) == 0);
		if(GPIO_ReadInputDataBit(PORT, SDA) == 1){
			data |= (0x80 >> i);
		}
		while (GPIO_ReadInputDataBit(PORT, SCL) == 1);
	}
	return data;
}

/*
* Function: recieveData
* Description: recieve main data from master
* Input:
*   None
* Ouput: 
*   recieveData: recieve each data frame from master
*/
uint8_t recieveData(){
	uint8_t recieve_Data = 0;
	configInputSda();
	for(uint8_t i = 0; i < 8; i++){
		while (GPIO_ReadInputDataBit(PORT, SCL) == 0);
		if (GPIO_ReadInputDataBit(PORT, SDA) == 1){
			recieve_Data |= 0x80>>i;
		}
		while (GPIO_ReadInputDataBit(PORT, SCL) == 1);
	}
	configOutputSda();
	GPIO_ResetBits(PORT, SDA);
	return recieve_Data;
}

/*
* Function: slaveI2cRecieve
* Description: recieve data from master
* Input:
*   data: save data from master
*		size: size of data variable
* Ouput: 
*   None
*/
void slaveI2cRecieve(uint8_t data[], uint8_t size){
	
	recieveData:
	waitstartI2c();
	
	uint8_t addressFrame = recieveAddress();
	address = addressFrame>>1;
	uint8_t bitRorW = addressFrame & 1;
	
	configOutputSda();
	if(address ==  SLAVE_ADDRESS) GPIO_ResetBits(PORT, SDA);
	else{
		GPIO_SetBits(PORT, SDA);
		while (GPIO_ReadInputDataBit(PORT, SCL) ==0);
		configInputSda();
		goto recieveData;
	}
	
	while(GPIO_ReadInputDataBit(PORT, SCL) == 0);
	if (bitRorW == 1){
		for(uint8_t	i = 0; i < size; i++){
			data[i] = recieveData();
		}
	}
}