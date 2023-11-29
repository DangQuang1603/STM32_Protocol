/*
*File: main.c
*Author: Le Dang Quang
*Date: 14/10/2023
*Disciption: This file is create to transmitt data by i2c communication
*/

/*--------------------------------CODE----------------------------------------*/

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

#define SLAVE_ADDRESS 0x58
#define SDA GPIO_Pin_6
#define SCL GPIO_Pin_7
#define PORT GPIOA
#define ACK 0
#define NACK 1

typedef enum{
	WRITE	= 1,	//write data to slave
	READ	= 0		//read data from slave
}mode;

void timer2Init();
void TIM2_IRQHandler();
void delayMs(uint32_t ms);
void configInputSda();
void configOutputSda();
void clock();
void configInit();
void startI2c();
void endI2c();
void transmitAddress();
void transmitData(uint8_t data);
uint8_t recieveData();
void masterI2cTransmit(uint8_t data[], uint8_t size, mode WorR);

uint32_t millisecond = 0;
uint8_t data[10] = {12,23,34,45,56,67,78,89,90,10};
uint8_t dataRecieve[10] = {0};


int main(){
	timer2Init();
	configInit();
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	delayMs(3000);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);

	masterI2cTransmit(data,10, READ);
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
*/
void configInputSda(){
	GPIOA->CRL &= ~(0xF<<24);
	GPIOA->CRH |= 1<<27;
	GPIOA->ODR |= 1<<6;
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
	GPIOA->CRL &= ~(0xF << 24);
	GPIOA->CRL |= 1<<25;
	GPIOA->ODR |= 1<<6;
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	initIcPort.GPIO_Pin = SDA|SCL|GPIO_Pin_13;
	initIcPort.GPIO_Mode = GPIO_Mode_Out_PP;
	initIcPort.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &initIcPort);
	GPIO_Init(GPIOC, &initIcPort);
	
	GPIO_SetBits(GPIOA, SDA|SCL);
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
* Function: startI2c
* Description: switch SDA line from High to Low voltage before SCL goes like this to generating Stop signal
* Input:
*   None
* Ouput: 
*   None
*/
void startI2c(){
	GPIO_ResetBits(GPIOA, SDA);
	delayMs(10);
	GPIO_ResetBits(GPIOA, SCL);
	delayMs(10);
}

/*
* Function: startI2c
* Description: switch SDA line from Low to High voltage after SCL goes like this to generating Stop signal
* Input:
*   None
* Ouput: 
*   None
*/
void endI2c(){
	GPIO_SetBits(GPIOA, SCL);
	GPIO_SetBits(GPIOA, SDA);
}

/*
* Function: transmitAddress
* Description: transmit address from master to all slave 
* Input:
*   None
* Ouput: 
*   None
*/
void transmitAddress(){
	for(uint8_t i = 0; i < 7; i++){
		if((0x58 & (0x40>>i)) == 0) GPIO_ResetBits(GPIOA,SDA);
		else GPIO_SetBits(GPIOA,SDA);
		clock();
	}
}

/*
* Function: transmitData
* Description: transmit data from master to identified slave 
* Input:
*   data: transmit data
* Ouput: 
*   None
*/
void transmitData(uint8_t data){
	for(uint8_t i = 0; i < 8; i++){
		if((data & 0x80>>i) == 0) GPIO_ResetBits(GPIOA,SDA);
		else GPIO_SetBits(GPIOA, SDA);
		clock();
	}
}

/*
* Function: recieveData
* Description: recieve data send from slave 
* Input:
*   None
* Ouput: 
*   None
*/
uint8_t recieveData(){
	uint8_t data;
	for(uint8_t i = 0; i < 8; i++){
		clock();
		if(GPIO_ReadInputDataBit(PORT,SDA) == 1) data |= (0x80>>i);
	}
}
/*
* Function: masterI2cTransmit
* Description: transmit all data from master to identified slave 
* Input:
*   data: transmit all data data 
* Ouput: 
*   None
*/
void masterI2cTransmit(uint8_t data[], uint8_t size, mode modeWorR){
	
	masterTrans:
	
	startI2c();
	transmitAddress();
	
	if(modeWorR == WRITE){
		GPIO_SetBits(GPIOA, SDA);
		clock();
		for(uint8_t i = 0; i < size; i++){
		configInputSda();
		if(GPIO_ReadInputDataBit(GPIOA,SDA) == ACK){
			configOutputSda();
			transmitData(data[i]);
		}else{
			configOutputSda();
			goto masterTrans;
			}
		}
		configInputSda();
		if(GPIO_ReadInputDataBit(GPIOA,SDA) == ACK){
			configOutputSda();
			endI2c();
		}else{
			configOutputSda();
			goto masterTrans;
		}
	}
	else if(modeWorR == READ){
		GPIO_ResetBits(GPIOA, SDA);
		clock();
		for(uint8_t i = 0; i < size; i++){
			configInputSda();
			data[i] = recieveData();
			if(GPIO_ReadInputDataBit(GPIOA,SDA) == ACK){}
			else if(GPIO_ReadInputDataBit(GPIOA,SDA) == NACK){
				configOutputSda();
				goto masterTrans;
			}
		}
		configInputSda();
		if(GPIO_ReadInputDataBit(GPIOA,SDA) == ACK){
			configOutputSda();
			endI2c();
		}else{
			configOutputSda();
			goto masterTrans;
		}
	}
	endI2c();
}
