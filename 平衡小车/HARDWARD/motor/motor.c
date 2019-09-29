#include "motor.h"

void Motor_Init(void)  //IO��ʼ��
{
	RCC->APB2ENR|=1<<3;       //PORTBʱ��ʹ��   
	GPIOB->CRH&=0X0000FFFF;   //PORTB12 13 14 15�������
	GPIOB->CRH|=0X22220000;   //PORTB12 13 14 15�����������
	
}

void PWM_Init(u16 arr,u16 psc)   ///time3 ��3  4ͨ��
{
	Motor_Init();
	RCC->APB1ENR|=1<<1;       //ʹ��TIM3ʱ��    
	RCC->APB2ENR|=1<<3;        //PORTBʱ��ʹ��  
	GPIOB->CRL&=0XFFFFFF00;    //PORTB0 �������    
	GPIOB->CRL|=0X000000BB;    //PORTB0 �������
	
	TIM3->ARR=arr;             //�趨�������Զ���װֵ 
	TIM3->PSC=psc;             //Ԥ��Ƶ������Ƶ
	
	TIM3->CCMR2|=6<<12;        //CH4 PWM1ģʽ	
	TIM3->CCMR2|=6<<4;         //CH3 PWM1ģʽ	
	
	TIM3->CCMR2|=1<<11;        //CH4Ԥװ��ʹ��	
	TIM3->CCMR2|=1<<3;         //CH3Ԥװ��ʹ��	 
	
	TIM3->CCER|=1<<12;         //CH4���ʹ��	   
	TIM3->CCER|=1<<8;          //CH3���ʹ��	
	
	//TIM3->BDTR |= 1<<15;       //TIM1����Ҫ��仰�������PWM
	
	TIM3->CR1|=1<<7;          //ARPEʹ�� 
	TIM3->CR1|=0x01;          //ʹ�ܶ�ʱ��3
	
}
