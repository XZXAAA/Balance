#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 

#define PWMA   TIM3->CCR3

#define PWMB   TIM3->CCR4


#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define BIN2   PBout(12)
#define BIN1   PBout(13)

void PWM_Init(u16 arr,u16 psc);


void Motor_Init(void);
#endif
