#include "encoder.h"

void Encoder_Init_TIM2(void)
{
	
	RCC->APB1ENR|=1<<0;     //TIM2ʱ��ʹ��
	RCC->APB2ENR|=1<<2;    //ʹ��PORTAʱ��
	GPIOA->CRL&=0XFFFFFF00;//PA0 PA1
	GPIOA->CRL|=0X00000044;//��������
	/* �Ѷ�ʱ����ʼ��Ϊ������ģʽ */ 
	TIM2->PSC = 0x0;//Ԥ��Ƶ�� ����Ƶ   //72MHZ
	TIM2->ARR = ENCODER_TIM_PERIOD-1;//�趨�������Զ���װֵ 
	
	
  TIM2->CCMR1 |= 1<<0;          //����ģʽ��IC1FP1ӳ�䵽TI1��
  TIM2->CCMR1 |= 1<<8;          //����ģʽ��IC2FP2ӳ�䵽TI2��
	
	
  TIM2->CCER |= 0<<1;           //IC1������
  TIM2->CCER |= 0<<5;           //IC2������
	
	
	TIM2->SMCR |= 3<<0;	          //SMS='011' ���е�������������غ��½�����Ч
	
	TIM2->CR1 |= 0x01;    //CEN=1��ʹ�ܶ�ʱ��
}

void Encoder_Init_TIM4(void)
{
	
	RCC->APB1ENR|=1<<2;     //TIM4ʱ��ʹ��
	RCC->APB2ENR|=1<<3;    //ʹ��PORTbʱ��
	GPIOB->CRL&=0X00FFFFFF;//PB6 PB7
	GPIOB->CRL|=0X44000000;//��������
	/* �Ѷ�ʱ����ʼ��Ϊ������ģʽ */ 
	TIM4->PSC = 0x0;//Ԥ��Ƶ��
	TIM4->ARR = ENCODER_TIM_PERIOD-1;//�趨�������Զ���װֵ 
  TIM4->CCMR1 |= 1<<0;          //����ģʽ��IC1FP1ӳ�䵽TI1��
  TIM4->CCMR1 |= 1<<8;          //����ģʽ��IC2FP2ӳ�䵽TI2��
  TIM4->CCER |= 0<<1;           //IC1������
  TIM4->CCER |= 0<<5;           //IC2������
	TIM4->SMCR |= 3<<0;	          //SMS='011' ���е�������������غ��½�����Ч
	TIM4->CR1 |= 0x01;    //CEN=1��ʹ�ܶ�ʱ��
}
