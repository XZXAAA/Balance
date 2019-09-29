#include "stdio.h"
#include "usart3.h"
u8 Usart3_Receive;
u8 mode_data[8];

u8 USART3_RX_BUF[20];     //接收缓冲,最大USART_REC_LEN个字节.

u16 USART3_RX_STA=0;       //接收状态标记	  
u8 flag;
u8 flag_Balance=1,flag_Velocity=1,flag_Turn=1,flag_Home,flag_Wave;
extern  int Encoder_Integral;
extern  	   int Velocity;
extern 		 int	Encoder,Turn_Amp;

extern  unsigned int Speed;
extern  long Turn_Bias_Integral;

u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //蓝牙遥控相关的变量

/**************************************************************************
函数功能：串口3初始化
入口参数：pclk2:PCLK2 时钟频率(Mhz)    bound:波特率
返回  值：无
**************************************************************************/


#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART3->SR&0X40)==0);//Flag_Show=0  使用串口3   
	USART3->DR = (u8) ch;      
	return ch;
}
#endif 






void uart3_init(u32 pclk2,u32 bound)
{  	 
	NVIC_InitTypeDef NVIC_InitStructure;
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<3;   //使能PORTB口时钟  
	RCC->APB1ENR|=1<<18;  //使能串口时钟 
	GPIOB->CRH&=0XFFFF00FF; 
	GPIOB->CRH|=0X00008B00;//IO状态设置
	GPIOB->ODR|=1<<10;	  
	RCC->APB1RSTR|=1<<18;   //复位串口1
	RCC->APB1RSTR&=~(1<<18);//停止复位	   	   
	//波特率设置
 	USART3->BRR=mantissa; // 波特率设置	 
	USART3->CR1|=0X200C;  //1位停止,无校验位.
	//使能接收中断
	USART3->CR1|=1<<8;    //PE中断使能
	USART3->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
//	MY_NVIC_Init(2,1,USART3_IRQn,2);//组2，最低优先级 

		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//抢占优先级2 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//响应优先级2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}


/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
void USART3_IRQHandler(void)
{	
	
	u8 res;	
	if(USART3->SR&(1<<5))	//接收到数据
	{	 
			res=USART3->DR; 
			USART3_RX_BUF[USART3_RX_STA]=res;
			if(res=='}'||(res>='A'&&res<='Z')||(res>='a'&&res<='z'))//接收到了帧尾
			{
				USART3_RX_STA=0;	//接收完成了		
				flag=1;
			}
			else //还没收完
			{	
					USART3_RX_STA++;
			}
		}  		 									     	
}

void USART_3(void)
{
		u8 *str;
		static int i; 
		unsigned int data;
		u8 select;
		if(flag)
		{	
			flag=0;
			if((int)USART3_RX_BUF[0]!='{')
		{					switch((int)USART3_RX_BUF[0])
					{
						case 'A':			Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;break;//////////////前
						case 'E':			Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;break;//////////////后
						case 'C':			Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;break;//////////////
						case 'G':			Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;break;//////////////
						case 'Z':			Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;break;//////////////
//						case 'a':			Flag_Stop=!Flag_Stop;break;
						case 'b':			printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",Balance_P,Balance_D,Velocity_P,Velocity_I,Turn_P,Turn_D,Turn_Amp,Speed,0);break;
						case 'c':			 if(flag_Balance)flag_Balance=0;  
														else  flag_Balance=1;		break;
						
						
					  case 'd':			if(flag_Velocity==1)
								{flag_Velocity=0;
							Encoder_Integral=0;
							Encoder=0;
							Velocity=0;}
							else
								flag_Velocity=1; 	break;
							
							
							
 						case 'e':			if(flag_Turn){flag_Turn=0;Turn_Bias_Integral=0;}
													else flag_Turn=1;  break;
//						case 'f':			flag_Home=!flag_Home;					break;
//						case 'g':			flag_Wave=!flag_Wave;					break;
//						case 'h':			intial();											break;
//						case 'i':			Flag_Bizhang=!Flag_Bizhang;		
//													if(Flag_Bizhang==1)TIM2_Cap_Init(0XFFFF,72-1); //=====如果Flag_Bizhang置1，则把TIM2初始化为超声波接口 然后不采集左路编码器，通过右路编码器和陀螺仪间接算出左路车轮速度
//													if(Flag_Bizhang==0)Encoder_Init2();	           //=====如果Flag_Bizhang置0，则把TIM2初始化为编码器接口
//													break;

						default:			break;
					}
		}
			if((int)USART3_RX_BUF[1]=='#')
		{
			str=USART3_RX_BUF;
			sscanf((char*)str,"{#%d:%d:%d:%d:%d:%d:%d:%d:%d}",&Balance_P,&Balance_D,&Velocity_P,&Velocity_I,&Turn_P,&Turn_D,&Turn_Amp,&Speed);//Amplitude
		}
			if((int)USART3_RX_BUF[2]==':')
		{
			USART3_RX_BUF[0]='0';
			select=USART3_RX_BUF[1];
			USART3_RX_BUF[1]='0';
			USART3_RX_BUF[2]='0';
			for(i=2;i<=20;i++)
				if(USART3_RX_BUF[i]=='}')
					USART3_RX_BUF[i]='\0';
			sscanf((char*)USART3_RX_BUF, "%d", &data);
			switch(select)
			{
				case'0':	Balance_P=(int)data;	break;
				case'1':	Balance_D=(int)data;	break;
				case'2':	Velocity_P=(int)data;	break;
				case'3':	Velocity_I=(int)data;	break;
				case'4':	Turn_P=(int)data;	break;
				case'5':	Turn_D=(int)data;	break;
				case'6':	Turn_Amp=(int)data;	break;
				case'7':	Speed=(int)data;	break;
			//	case'8':	T=(int)data;	break;
				default:	break;
			}
			data=0;
		}
		USART3_RX_STA=0;
		for(i=0;i<=20;i++)	
			USART3_RX_BUF[i]=0;
	}
}


/**************************************************************************
函数功能：往app发送数据
入口参数：无
返回  值：无
作    者：
**************************************************************************/
