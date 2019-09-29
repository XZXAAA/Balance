#include "stm32f10x.h"
#include "delay.h"
#include  "encoder.h"
#include "usart.h"
#include "motor.h"
#include "timer.h"
#include "exti.h"
#include "oled.h"
#include "inv_mpu.h"
#include "mpu6050.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "usart3.h"
#include "stdio.h"
#include "adc.h"
extern int Moto1;
extern int target;

short gyrox,gyroy,gyroz;  //陀螺仪的数据
short aacx,aacy,aacz;		//加速度传感器原始数据
extern const unsigned char test1[2][32];
extern float Angle_Balance;
extern int Encoder_Left  ;
extern int Encoder_Right ;
extern  int Encoder_Least;
extern int Moto1;
extern int	Encoder;

extern  	   int Velocity;
extern	   int Encoder_Integral;
extern unsigned int Balance_P,Balance_D,Velocity_P,Velocity_I,Turn_P,Turn_D,Speed;

extern  float Bias;
extern u8 flag_Velocity ;
extern u8 flag_Balance;
extern u8 flag_Turn;

 void Delay(u32 count)
 {
  u32 i=0;
  for(;i<count;i++);

 }
 int main(void)
 {	
	 char t[30];

	 float pitch,roll,yaw; 		//欧拉角
	 int i =0;
	 
	 
	     char a[] = "SF-TECH";
			a++;
    printf("%s", a);
	 
	 
	 
	delay_init();	  //延时函数初始化
	 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	 EXTIX_Init();
	 OLED_Init();	
  uart_init(9600);
	 	uart3_init(36,9600);            //=====串口3初始化 波特率：9600 //蓝牙
	 Adc_Init();
	PWM_Init(3599,0);   //=====初始化PWM 10KHZ 高频可以防止电机低频时的尖叫声   周期是0.1ms
	Encoder_Init_TIM2();
	Encoder_Init_TIM4();
	 MPU_Init();
	 
	 
	 //调完  //师兄小车
//	Velocity_P=44;
//	 Velocity_I=22;
//	 Balance_P=219;
//	 Balance_D=520;
	 
		Balance_P=303;    //Balance_P=378;    //*0.6  328   970  124  9
		Balance_D=900;    //Balance_D=367;    //*0.6
		Velocity_P=105;
		Velocity_I=8;                         ///255 577  104 3
		
		Turn_P=20;
		Turn_D=3;
	 while( mpu_dmp_init())
		printf("MPU6050 ERROR\r\n  \r\r");
	 
	 	Timer1_Init(49,7199);   ///50ms

	 while(1)
		 {
			
			 		
			 
			sprintf(t,"Angle:%f",Angle_Balance);
			OLED_ShowString(0,35,t,12);
			// sprintf(t,":%d  :%d \r\n",Moto1,Encoder_Least);
			//OLED_ShowString(0,50,t,12);
			 sprintf(t,"V_P:%d, V_I:%d",Velocity_P,Velocity_I);
			OLED_ShowString(0,20,t,12);
			 sprintf(t,"B_P:%d B_D:%d ",Balance_P,Balance_D);
			OLED_ShowString(0,5,t,12);
			 sprintf(t,"T_L:%d V_L:%d B_L:%d ",flag_Turn,flag_Velocity,flag_Balance );
			OLED_ShowString(0,50,t,12);
			
			OLED_Refresh_Gram();
			 i++;
			 if(i==500)
			 {
				 OLED_Clear();
				 OLED_Refresh_Gram();
				 i=0;
			 }
				 
			 USART_3();
			 }
			
			 
	

 }
 


