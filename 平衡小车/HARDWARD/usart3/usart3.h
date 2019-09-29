#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h" 	

extern u8  USART3_RX_BUF[20]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 

extern u16 USART3_RX_STA;         		//接收状态标记	

extern unsigned int Balance_P,Balance_D,Velocity_P,Velocity_I,Turn_P,Turn_D,Speed;
//extern u8 flag_Balance,flag_Velocity,flag_Turn,flag_Home,flag_Wave;
extern int Amplitude;
extern u8 Usart3_Receive;
void uart3_init(u32 pclk2,u32 bound);
void USART_3(void);

void USART3_IRQHandler(void);
#endif

