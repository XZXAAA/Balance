#ifndef __TIMER_H
#define __TIMER_H
#include <sys.h>	 

void Timer4_Init(u16 arr,u16 psc);  
void Timer1_Init(u16 arr,u16 psc); 
int Incremental_PI (int Encoder,int Target);
int Incremental_PI_2 (int Encoder,int Target);
int myabs(int a);
void Set_Pwm(int moto1,int moto2);
void Xianfu_Pwm(void);

void Get_Angle(void);
void Yijielvbo(float angle_m, float gyro_m);
int balance(float Angle,float Gyro);
///void SET_BALANCE_PWN(void);
int velocity(int encoder_left,int encoder_right);
int turn(int encoder_left,int encoder_right,float gyro);     //×ªÏò¿ØÖÆ
  
#endif
