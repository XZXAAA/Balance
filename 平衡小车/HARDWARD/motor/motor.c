#include "motor.h"

void Motor_Init(void)  //IO初始化
{
	RCC->APB2ENR|=1<<3;       //PORTB时钟使能   
	GPIOB->CRH&=0X0000FFFF;   //PORTB12 13 14 15推挽输出
	GPIOB->CRH|=0X22220000;   //PORTB12 13 14 15推挽输出？？
	
}

void PWM_Init(u16 arr,u16 psc)   ///time3 的3  4通道
{
	Motor_Init();
	RCC->APB1ENR|=1<<1;       //使能TIM3时钟    
	RCC->APB2ENR|=1<<3;        //PORTB时钟使能  
	GPIOB->CRL&=0XFFFFFF00;    //PORTB0 复用输出    
	GPIOB->CRL|=0X000000BB;    //PORTB0 复用输出
	
	TIM3->ARR=arr;             //设定计数器自动重装值 
	TIM3->PSC=psc;             //预分频器不分频
	
	TIM3->CCMR2|=6<<12;        //CH4 PWM1模式	
	TIM3->CCMR2|=6<<4;         //CH3 PWM1模式	
	
	TIM3->CCMR2|=1<<11;        //CH4预装载使能	
	TIM3->CCMR2|=1<<3;         //CH3预装载使能	 
	
	TIM3->CCER|=1<<12;         //CH4输出使能	   
	TIM3->CCER|=1<<8;          //CH3输出使能	
	
	//TIM3->BDTR |= 1<<15;       //TIM1必须要这句话才能输出PWM
	
	TIM3->CR1|=1<<7;          //ARPE使能 
	TIM3->CR1|=0x01;          //使能定时器3
	
}
