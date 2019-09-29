#include "stdio.h"
#include "usart3.h"
u8 Usart3_Receive;
u8 mode_data[8];

u8 USART3_RX_BUF[20];     //���ջ���,���USART_REC_LEN���ֽ�.

u16 USART3_RX_STA=0;       //����״̬���	  
u8 flag;
u8 flag_Balance=1,flag_Velocity=1,flag_Turn=1,flag_Home,flag_Wave;
extern  int Encoder_Integral;
extern  	   int Velocity;
extern 		 int	Encoder,Turn_Amp;

extern  unsigned int Speed;
extern  long Turn_Bias_Integral;

u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //����ң����صı���

/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ�����pclk2:PCLK2 ʱ��Ƶ��(Mhz)    bound:������
����  ֵ����
**************************************************************************/


#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART3->SR&0X40)==0);//Flag_Show=0  ʹ�ô���3   
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
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<3;   //ʹ��PORTB��ʱ��  
	RCC->APB1ENR|=1<<18;  //ʹ�ܴ���ʱ�� 
	GPIOB->CRH&=0XFFFF00FF; 
	GPIOB->CRH|=0X00008B00;//IO״̬����
	GPIOB->ODR|=1<<10;	  
	RCC->APB1RSTR|=1<<18;   //��λ����1
	RCC->APB1RSTR&=~(1<<18);//ֹͣ��λ	   	   
	//����������
 	USART3->BRR=mantissa; // ����������	 
	USART3->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART3->CR1|=1<<8;    //PE�ж�ʹ��
	USART3->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
//	MY_NVIC_Init(2,1,USART3_IRQn,2);//��2��������ȼ� 

		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//��ռ���ȼ�2 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//��Ӧ���ȼ�2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
		NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}


/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void USART3_IRQHandler(void)
{	
	
	u8 res;	
	if(USART3->SR&(1<<5))	//���յ�����
	{	 
			res=USART3->DR; 
			USART3_RX_BUF[USART3_RX_STA]=res;
			if(res=='}'||(res>='A'&&res<='Z')||(res>='a'&&res<='z'))//���յ���֡β
			{
				USART3_RX_STA=0;	//���������		
				flag=1;
			}
			else //��û����
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
						case 'A':			Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;break;//////////////ǰ
						case 'E':			Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;break;//////////////��
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
//													if(Flag_Bizhang==1)TIM2_Cap_Init(0XFFFF,72-1); //=====���Flag_Bizhang��1�����TIM2��ʼ��Ϊ�������ӿ� Ȼ�󲻲ɼ���·��������ͨ����·�������������Ǽ�������·�����ٶ�
//													if(Flag_Bizhang==0)Encoder_Init2();	           //=====���Flag_Bizhang��0�����TIM2��ʼ��Ϊ�������ӿ�
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
�������ܣ���app��������
��ڲ�������
����  ֵ����
��    �ߣ�
**************************************************************************/
