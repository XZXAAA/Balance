#include "timer.h"
#include "motor.h"
#include "stdio.h"
#include "oled.h"
#include "inv_mpu.h"
#include "mpu6050.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
void Get_battery_volt(void);
void 	AppDataScope(void);
int Voltage = 0 ;
int Encoder_Left =0 ;
int Encoder_Right =0;
int target = -10;  //30��*4��Ƶvel    vel   = 50    150->  1r     rpm*20*10^-3    =  target

int Turn_Amp=1000;
extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //����ң����صı���

 int Encoder_Least=0;
 	   int Velocity=0;
	   int Encoder_Integral=0;
		 int	Encoder=0;
		 int Movement=0;
		 unsigned int Speed=25;
		 
		  float Bias;
 long Turn_Bias_Integral=0;
extern u8 flag_Velocity ;
extern u8 flag_Balance,flag_Turn;
int Moto1=0;
int Moto2=0;

float K1 =0.02; 
float angle, angle_dot;
float Gyro_Turn;

extern short gyrox,gyroy,gyroz;  //�����ǵ�����
extern short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����



int Balance_Pwm,Velocity_Pwm=0,Turn_Pwm;
int Amplitude =3500;    //===PWM������3600 ������3500
unsigned int Balance_P,Balance_D ,Turn_P,Turn_D,Speed;
unsigned int Velocity_P,Velocity_I;
//200 500

#define PI 3.14159265
float Angle_Balance,Gyro_Balance;
         //ƽ����� ƽ�������� 



void Timer4_Init(u16 arr,u16 psc)  
{  	  
		NVIC_InitTypeDef NVIC_InitStructure; 

		RCC->APB1ENR|=1<<2; //ʱ��ʹ��    
		TIM4->ARR=arr;      //�趨�������Զ���װֵ   
		TIM4->PSC=psc;      //Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��
		TIM4->DIER|=1<<0;   //��������ж�				
		TIM4->DIER|=1<<6;   //�������ж�	   



		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;		
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
		NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���


	
		TIM4->CR1|=0x01;    //ʹ�ܶ�ʱ��
	
}  


void Timer1_Init(u16 arr,u16 psc)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC->APB2ENR|=1<<11;//TIM2ʱ��ʹ��    
 	TIM1->ARR=arr;  //�趨�������Զ���װֵ//�պ�1ms    
	TIM1->PSC=psc;  //Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��
	TIM1->DIER|=1<<0;   //��������ж�				
	TIM1->DIER|=1<<6;   //�������ж�	

		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;		
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
		NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	
	TIM1->CR1|=0x01;    //ʹ�ܶ�ʱ��
}



void  TIM1_UP_IRQHandler()
{
	 static int num = 200;
	
			if(TIM1->SR&0X0001)//10ms��ʱ�ж�
	{   

		TIM1->SR&=~(1<<0);   
                                    //===�����ʱ��3�жϱ�־λ
		
		
		Encoder_Right = (short)TIM2 -> CNT;    //�ѱ�������ֵ������
		TIM2 -> CNT=0;   //���Ϊ0
		
		Encoder_Left = (short)TIM4 -> CNT;    //�ѱ�������ֵ������
		TIM4 -> CNT=0;   //���Ϊ0
			
		Get_Angle();  //�õ��ǶȺͽ��ٶ�
//////ʦ��С��
//		if(flag_Balance==1)
//    Balance_Pwm = balance(Angle_Balance,Gyro_Balance);   //���ݽǶȽ��ٶȽ���PID
//		else Balance_Pwm=0;
//		
//		if(flag_Velocity==1)
//		Velocity_Pwm = velocity(Encoder_Left,Encoder_Right);//�ٶ�PID
//		else
//			Velocity_Pwm=0;
//		
//		if(flag_Turn==1)
//			Turn_Pwm = turn(Encoder_Left,Encoder_Right,Gyro_Turn);
//		else 
//			Turn_Pwm=0;
//		Moto1=Balance_Pwm-Velocity_Pwm+Turn_Pwm;   //+
//		Moto2=Balance_Pwm-Velocity_Pwm-Turn_Pwm;    //+
		
		
		if(flag_Balance==1)
    Balance_Pwm = balance(Angle_Balance,Gyro_Balance);   //���ݽǶȽ��ٶȽ���PID
		else Balance_Pwm=0;
		
		if(flag_Velocity==1)
		Velocity_Pwm = velocity(Encoder_Left,Encoder_Right);//�ٶ�PID
		else
			Velocity_Pwm=0;
		
		if(flag_Turn==1)
			Turn_Pwm = turn(Encoder_Left,Encoder_Right,Gyro_Turn);
		else 
			Turn_Pwm=0;
		
		Moto1=-Balance_Pwm+Velocity_Pwm+Turn_Pwm;   //+
		Moto2=-Balance_Pwm+Velocity_Pwm-Turn_Pwm;    //+
		
		Xianfu_Pwm();
		
		Set_Pwm(Moto1,Moto2);
		
		if((num--)<0)
		{
			Get_battery_volt();   //����ѹ
			AppDataScope();
			num = 200;
		}
		


	
	}
}



void AppDataScope(void)
{   
		static int l,r;
		if(1)	
		{
				if(Moto1>0)
						l=(int)(myabs(Moto1)/70.0)+50;
				else
						l=50-(int)(myabs(Moto1)/70.0);
				
				if(Moto2>0)
						r=(int)(myabs(Moto2)/70.0)+50;
				else
						r=50-(int)(myabs(Moto2)/70.0);			
				
				printf("{A%d:%d:%d:%d}$",l,r,(int)(Voltage/12.60),(int)Angle_Balance);//��ӡ��APP����
		}
		//if(1)	
		//	printf("{B%d:%d:%d:%d}$",(int)Angle_Balance,(int)Gyro_Balance,(int)Gyro_Turn,(int)Turn_Pwm);//��ӡ��APP����
}

int velocity(int encoder_left,int encoder_right)
{

		
				if(Flag_Qian==1)	Movement=Speed;	             //===���ǰ����־λ��1 λ��Ϊ��
		else if(Flag_Hou==1)	  Movement=-Speed;          //===������˱�־λ��1 λ��Ϊ��
	  else  Movement=0;	

   //=============�ٶ�PI������======================//	
		Encoder_Least =(Encoder_Left + Encoder_Right)  -  0;  //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder = Encoder * 0.8;		                             //===һ�׵�ͨ�˲���       
		Encoder = Encoder + Encoder_Least*0.2;	                 //===һ�׵�ͨ�˲���    
	
	

	
		Encoder_Integral += Encoder;                     //===���ֳ�λ�� ����ʱ�䣺5ms
	  Encoder_Integral=Encoder_Integral+Movement; 
	
		if(Encoder_Integral>360000)  	Encoder_Integral=360000;          //===�����޷�
		if(Encoder_Integral<-360000)	Encoder_Integral=-360000;         //===�����޷�	
	
//		Velocity =(int)((int)Encoder*(int)Velocity_P+((int)Encoder_Integral/(1.0*(int)Velocity_I)));	//===�ٶ�PI������	�ٶ�Pϵ����4.2  Iϵ����1/110
		Velocity =(int)((int)Encoder*(int)Velocity_P+((int)Encoder_Integral*2.0/(1.0*(int)Velocity_I)));	//===�ٶ�PI������	�ٶ�Pϵ����4.2  Iϵ����1/110
	
	
	  return Velocity;
	
}
/**************************************************************************
�������ܣ�ת��PD����
��ڲ��������ֱ����������ֱ�������Z��������
����  ֵ��ת�����PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//ת�����
{
  	static int Turn_Target,Turn,Encoder_temp,Turn_Convert=3,Turn_Count;
	  int Turn_Bias,Turn_Amplitude=(1500/3)-300;   //+800  //===Way_AngleΪ�˲�����������1ʱ������DMP��ȡ��̬��Turn_Amplitudeȡ�󣬿������ͻ����ǣ�ȡС����Ϊ�������˲��㷨Ч���Բ
	  
	  //=============ң��������ת����=======================//
  	if(Flag_Left==1||Flag_Right==1)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(encoder_left+encoder_right);
			Turn_Convert=2000/Encoder_temp;
			if(Turn_Convert<3)Turn_Convert=3;
			else if(Turn_Convert>10)Turn_Convert=10;
		}	
	  else
		{
			Turn_Convert=3;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(1==Flag_Left)	           Turn_Target+=Turn_Convert; //��ת
		else if(1==Flag_Right)	     Turn_Target-=Turn_Convert; //��ת
		else Turn_Target=0;                                     //ֹͣ
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
  	//=============ת��PD������=======================//
		 Turn_Bias=Encoder_Left-Encoder_Right;         //===����ת���ٶ�ƫ��  
		if(1)         //Ϊ�˷�ֹ����Ӱ���û����飬ֻ�е��������ʱ��ſ�ʼ����
		{	
		Turn_Bias_Integral+=Turn_Bias;                //ת���ٶ�ƫ����ֵõ�ת��ƫ������������仰��
		Turn_Bias_Integral-=Turn_Target;              //��ȡң��������
		}
		if(Turn_Bias_Integral>10)  	Turn_Bias_Integral=10;          //===�����޷�
		else if(Turn_Bias_Integral<-10)	Turn_Bias_Integral=-10;     //===�����޷�	
		Turn= (int)((int)Turn_Bias_Integral* (int)Turn_P- (float)(gyro*1.0/(Turn_D*1.0)));						//===���Z�������ǽ���PD����
		
		if(Turn>Turn_Amp)  	Turn=Turn_Amp;          //
		else if(Turn<-Turn_Amp)	Turn=-Turn_Amp;     //	
		
		
		if(Flag_Left==1||Flag_Right==1)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
		{
			return Turn;
		}
		else return 0;
}

void Set_Pwm(int moto1,int moto2)
{
			if(moto1<0)			AIN2=1,			AIN1=0;
			else 	          AIN2=0,			AIN1=1;  //��
			
			if(moto2>0)			BIN2=1,			BIN1=0;
			else 	          BIN2=0,			BIN1=1;
			PWMA=myabs(moto1);
			PWMB=myabs(moto2);
}




//int Incremental_PI (int Encoder,int Target)  ///PID�㷨
//{ 	
//   float Kp=20,Ki=30;	  //KI=30
//	 static int Bias,Pwm,Last_bias;
//	 Bias=Encoder-Target;                //����ƫ��
//	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //����ʽPI������
//	 Last_bias=Bias;	                   //������һ��ƫ�� 
//	 return Pwm;                         //�������    ���ֵ����Ҫ��ֵ��CCR��ֵ
//}

//int Incremental_PI_2 (int Encoder,int Target)  ///PID�㷨
//{ 	
//   float Kp=20,Ki=30;	  //KI=30
//	 static int Bias,Pwm,Last_bias;
//	 Bias=Encoder-Target;                //����ƫ��
//	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //����ʽPI������
//	 Last_bias=Bias;	                   //������һ��ƫ�� 
//	 return Pwm;                         //�������    ���ֵ����Ҫ��ֵ��CCR��ֵ
//}


int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


void Xianfu_Pwm()
{	
	      //===PWM������7200 ������7100
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	
	   if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;	
}


void Get_Angle()
{ 
	
	    float Accel_Y;		
    
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
					
			
			
	
			Gyro_Balance=-gyroy;                                  //����ƽ����ٶ�///ע����ת��ķ���
	   	Accel_Y=atan2(aacx,aacz)*180/PI;                 //���������ļн�	
		  gyroy=gyroy/16.4;                                    //����������ת��	 �ֱ���     ��λ��/s

				Yijielvbo(Accel_Y,-gyroy);
	    Angle_Balance=angle;                                   //����ƽ�����
			Gyro_Turn=gyroz;   


	  
}

void Yijielvbo(float angle_m, float gyro_m)
{
   angle = K1 * angle_m+ (0.98) * (angle + gyro_m * 0.005);
}

int balance(float Angle,float Gyro)
{  
  
	 int balance;
	 Bias=Angle-1.0;              //===���ƽ��ĽǶ���ֵ �ͻ�е��� -0��ζ������������0�ȸ��� �������������5�ȸ��� �Ǿ�Ӧ�ü�ȥ5
	 balance=(int)(Balance_P*Bias+Gyro*Balance_D/1000.0);   //===����ƽ����Ƶĵ��PWM  PD����   35��Pϵ�� 0.125��Dϵ��   //1000
	 return balance;
}




