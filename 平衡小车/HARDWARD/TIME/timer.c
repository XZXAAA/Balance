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
int target = -10;  //30线*4分频vel    vel   = 50    150->  1r     rpm*20*10^-3    =  target

int Turn_Amp=1000;
extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //蓝牙遥控相关的变量

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

extern short gyrox,gyroy,gyroz;  //陀螺仪的数据
extern short aacx,aacy,aacz;		//加速度传感器原始数据



int Balance_Pwm,Velocity_Pwm=0,Turn_Pwm;
int Amplitude =3500;    //===PWM满幅是3600 限制在3500
unsigned int Balance_P,Balance_D ,Turn_P,Turn_D,Speed;
unsigned int Velocity_P,Velocity_I;
//200 500

#define PI 3.14159265
float Angle_Balance,Gyro_Balance;
         //平衡倾角 平衡陀螺仪 



void Timer4_Init(u16 arr,u16 psc)  
{  	  
		NVIC_InitTypeDef NVIC_InitStructure; 

		RCC->APB1ENR|=1<<2; //时钟使能    
		TIM4->ARR=arr;      //设定计数器自动重装值   
		TIM4->PSC=psc;      //预分频器7200,得到10Khz的计数时钟
		TIM4->DIER|=1<<0;   //允许更新中断				
		TIM4->DIER|=1<<6;   //允许触发中断	   



		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;		
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器


	
		TIM4->CR1|=0x01;    //使能定时器
	
}  


void Timer1_Init(u16 arr,u16 psc)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC->APB2ENR|=1<<11;//TIM2时钟使能    
 	TIM1->ARR=arr;  //设定计数器自动重装值//刚好1ms    
	TIM1->PSC=psc;  //预分频器7200,得到10Khz的计数时钟
	TIM1->DIER|=1<<0;   //允许更新中断				
	TIM1->DIER|=1<<6;   //允许触发中断	

		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;		
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
	TIM1->CR1|=0x01;    //使能定时器
}



void  TIM1_UP_IRQHandler()
{
	 static int num = 200;
	
			if(TIM1->SR&0X0001)//10ms定时中断
	{   

		TIM1->SR&=~(1<<0);   
                                    //===清除定时器3中断标志位
		
		
		Encoder_Right = (short)TIM2 -> CNT;    //把编码器的值读出来
		TIM2 -> CNT=0;   //清除为0
		
		Encoder_Left = (short)TIM4 -> CNT;    //把编码器的值读出来
		TIM4 -> CNT=0;   //清除为0
			
		Get_Angle();  //得到角度和角速度
//////师兄小车
//		if(flag_Balance==1)
//    Balance_Pwm = balance(Angle_Balance,Gyro_Balance);   //根据角度角速度进行PID
//		else Balance_Pwm=0;
//		
//		if(flag_Velocity==1)
//		Velocity_Pwm = velocity(Encoder_Left,Encoder_Right);//速度PID
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
    Balance_Pwm = balance(Angle_Balance,Gyro_Balance);   //根据角度角速度进行PID
		else Balance_Pwm=0;
		
		if(flag_Velocity==1)
		Velocity_Pwm = velocity(Encoder_Left,Encoder_Right);//速度PID
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
			Get_battery_volt();   //读电压
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
				
				printf("{A%d:%d:%d:%d}$",l,r,(int)(Voltage/12.60),(int)Angle_Balance);//打印到APP上面
		}
		//if(1)	
		//	printf("{B%d:%d:%d:%d}$",(int)Angle_Balance,(int)Gyro_Balance,(int)Gyro_Turn,(int)Turn_Pwm);//打印到APP上面
}

int velocity(int encoder_left,int encoder_right)
{

		
				if(Flag_Qian==1)	Movement=Speed;	             //===如果前进标志位置1 位移为负
		else if(Flag_Hou==1)	  Movement=-Speed;          //===如果后退标志位置1 位移为正
	  else  Movement=0;	

   //=============速度PI控制器======================//	
		Encoder_Least =(Encoder_Left + Encoder_Right)  -  0;  //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此次为零） 
		Encoder = Encoder * 0.8;		                             //===一阶低通滤波器       
		Encoder = Encoder + Encoder_Least*0.2;	                 //===一阶低通滤波器    
	
	

	
		Encoder_Integral += Encoder;                     //===积分出位移 积分时间：5ms
	  Encoder_Integral=Encoder_Integral+Movement; 
	
		if(Encoder_Integral>360000)  	Encoder_Integral=360000;          //===积分限幅
		if(Encoder_Integral<-360000)	Encoder_Integral=-360000;         //===积分限幅	
	
//		Velocity =(int)((int)Encoder*(int)Velocity_P+((int)Encoder_Integral/(1.0*(int)Velocity_I)));	//===速度PI控制器	速度P系数是4.2  I系数是1/110
		Velocity =(int)((int)Encoder*(int)Velocity_P+((int)Encoder_Integral*2.0/(1.0*(int)Velocity_I)));	//===速度PI控制器	速度P系数是4.2  I系数是1/110
	
	
	  return Velocity;
	
}
/**************************************************************************
函数功能：转向PD控制
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
作    者：平衡小车之家
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
  	static int Turn_Target,Turn,Encoder_temp,Turn_Convert=3,Turn_Count;
	  int Turn_Bias,Turn_Amplitude=(1500/3)-300;   //+800  //===Way_Angle为滤波方法，当是1时，即由DMP获取姿态，Turn_Amplitude取大，卡尔曼和互补是，取小，因为这两种滤波算法效果稍差。
	  
	  //=============遥控左右旋转部分=======================//
  	if(Flag_Left==1||Flag_Right==1)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
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
		if(1==Flag_Left)	           Turn_Target+=Turn_Convert; //左转
		else if(1==Flag_Right)	     Turn_Target-=Turn_Convert; //右转
		else Turn_Target=0;                                     //停止
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
  	//=============转向PD控制器=======================//
		 Turn_Bias=Encoder_Left-Encoder_Right;         //===计算转向速度偏差  
		if(1)         //为了防止积分影响用户体验，只有电机开启的时候才开始积分
		{	
		Turn_Bias_Integral+=Turn_Bias;                //转向速度偏差积分得到转向偏差（请认真理解这句话）
		Turn_Bias_Integral-=Turn_Target;              //获取遥控器数据
		}
		if(Turn_Bias_Integral>10)  	Turn_Bias_Integral=10;          //===积分限幅
		else if(Turn_Bias_Integral<-10)	Turn_Bias_Integral=-10;     //===积分限幅	
		Turn= (int)((int)Turn_Bias_Integral* (int)Turn_P- (float)(gyro*1.0/(Turn_D*1.0)));						//===结合Z轴陀螺仪进行PD控制
		
		if(Turn>Turn_Amp)  	Turn=Turn_Amp;          //
		else if(Turn<-Turn_Amp)	Turn=-Turn_Amp;     //	
		
		
		if(Flag_Left==1||Flag_Right==1)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
		{
			return Turn;
		}
		else return 0;
}

void Set_Pwm(int moto1,int moto2)
{
			if(moto1<0)			AIN2=1,			AIN1=0;
			else 	          AIN2=0,			AIN1=1;  //右
			
			if(moto2>0)			BIN2=1,			BIN1=0;
			else 	          BIN2=0,			BIN1=1;
			PWMA=myabs(moto1);
			PWMB=myabs(moto2);
}




//int Incremental_PI (int Encoder,int Target)  ///PID算法
//{ 	
//   float Kp=20,Ki=30;	  //KI=30
//	 static int Bias,Pwm,Last_bias;
//	 Bias=Encoder-Target;                //计算偏差
//	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //增量式PI控制器
//	 Last_bias=Bias;	                   //保存上一次偏差 
//	 return Pwm;                         //增量输出    这个值就是要赋值给CCR的值
//}

//int Incremental_PI_2 (int Encoder,int Target)  ///PID算法
//{ 	
//   float Kp=20,Ki=30;	  //KI=30
//	 static int Bias,Pwm,Last_bias;
//	 Bias=Encoder-Target;                //计算偏差
//	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //增量式PI控制器
//	 Last_bias=Bias;	                   //保存上一次偏差 
//	 return Pwm;                         //增量输出    这个值就是要赋值给CCR的值
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
	      //===PWM满幅是7200 限制在7100
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	
	   if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;	
}


void Get_Angle()
{ 
	
	    float Accel_Y;		
    
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
					
			
			
	
			Gyro_Balance=-gyroy;                                  //更新平衡角速度///注意旋转轴的方向
	   	Accel_Y=atan2(aacx,aacz)*180/PI;                 //计算与地面的夹角	
		  gyroy=gyroy/16.4;                                    //陀螺仪量程转换	 分辨率     单位°/s

				Yijielvbo(Accel_Y,-gyroy);
	    Angle_Balance=angle;                                   //更新平衡倾角
			Gyro_Turn=gyroz;   


	  
}

void Yijielvbo(float angle_m, float gyro_m)
{
   angle = K1 * angle_m+ (0.98) * (angle + gyro_m * 0.005);
}

int balance(float Angle,float Gyro)
{  
  
	 int balance;
	 Bias=Angle-1.0;              //===求出平衡的角度中值 和机械相关 -0意味着身重中心在0度附近 如果身重中心在5度附近 那就应该减去5
	 balance=(int)(Balance_P*Bias+Gyro*Balance_D/1000.0);   //===计算平衡控制的电机PWM  PD控制   35是P系数 0.125是D系数   //1000
	 return balance;
}




