#include "include.h"
#include "ultrasonic.h"
#include "usart.h"
#include "filter.h"
#include "IMU.h"
#include "rc.h"
#include "ms5611.h"
#include "myiic_sonar.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "error.h"
float T_sonar;

void SONAR_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;
	//设置LED使用到得管脚
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	/*开启GPIOB的外设时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOF时钟
	/*选择要控制的GPIOC引脚*/															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11	;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉

	/*调用库函数，初始化GPIOB*/
	GPIO_Init(GPIOB, &GPIO_InitStructure);		 

	/*选择要控制的GPIOC引脚*/															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
	/*调用库函数，初始化GPIOB*/
	GPIO_Init(GPIOB, &GPIO_InitStructure);		 
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);//PE2 连接到中断线2
	/* 配置EXTI_Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;//LINE0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
	EXTI_Init(&EXTI_InitStructure);//配置

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//外部中断4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
}
void GPIO_SET_SONAR(u8 num)
{

GPIO_SetBits(GPIOB,GPIO_Pin_10);	
}

void GPIO_RESET_SONAR(u8 num)
{
GPIO_ResetBits(GPIOB,GPIO_Pin_10);
}

u8 GPIO_READ_SONAR(u8 num)
{u8 temp=0;
temp=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
return temp;
}

void Ultrasonic_Init()
{ 
	#if defined(SONAR_USE_UART)   
	//Uart5_Init(9600);			//串口5初始化，函数参数为波特率
	#endif
	#if defined(SONAR_USE_TIG)    
	SONAR_GPIO_Config();	ultra_ok = 1;
	#endif
	#if defined(SONAR_USE_SCL)    
	IIC_Init_Sonar();
	//KS10X_Change_Addr_Init(SlaveAddress3,SlaveAddress2);  
	//ultra_ok=AT24CXX_Check();	
	#endif
}

s8 ultra_start_f;
u8 ultra_time;
u8 ultra_ok = 0;
void Ultra_Duty()
{
	u8 temp[3];

	ultra_time++;
	ultra_time = ultra_time%2;
	
#if defined (SONAR_USE_UART)
		#if defined(USE_KS103)
			Uart5_Send(temp ,0xe8);Delay_us(20);
			Uart5_Send(temp ,0x02);Delay_us(20);
				 #if   defined(SONAR_SAMPLE1)
					Uart5_Send(temp ,0xb0);
				 #elif defined(SONAR_SAMPLE2)
					Uart5_Send(temp ,0xbc);
				 #elif defined(SONAR_SAMPLE3)
					Uart5_Send(temp ,0xb8);
				 #endif
	  #elif defined(URM07)
			Uart5_Send(temp ,0x55);Delay_us(20);
			Uart5_Send(temp ,0xAA);Delay_us(20);
			Uart5_Send(temp ,0x11);Delay_us(20);
	    Uart5_Send(temp ,0x00);Delay_us(20);
			Uart5_Send(temp ,0x02);Delay_us(20);
      Uart5_Send(temp ,0x12);Delay_us(20);
		#elif defined(USE_US100)
			temp[0] = 0x55;
			#if USE_IMU_BACK_IO_AS_SONAR
			while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
			USART_SendData(UART4, temp[0]); 
			#else
			while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
			USART_SendData(USART3, temp[0]); 
			#endif
		#endif
#else
	GPIO_SET_SONAR(0);
	delay_us(20);
	GPIO_RESET_SONAR(0);
#endif	
			
	ultra_start_f = 1;
}

// float t1r=1;
/* kalman filter states */
double x_pred = 0.0f; // m   0
double v_pred = 0.0f; //       1
double x_post = 0.0f; // m    2
double v_post = 0.0f; // m/s  3
float sonar_raw = 0.0f;  // m
float scale_kal_sonar_v=0.2;
float sonar_filter(float hight,float dt_sonar)
{float x_new;
 static float reg;
	float LPF_1=1;//0.75; 
	float MAX_SPEED=0.5;
	/* no data for long time */
	if (dt_sonar > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt_sonar * v_pred;
	v_pred = v_post;
   v_pred=limit_mine	(v_pred,MAX_SPEED);
	 if(fabs(v_pred) < 0.01)           \
    v_pred = 0;   
	 
	 v_pred=reg*(1-LPF_1)+v_pred*(LPF_1);
	 reg=v_pred;
	 x_new = hight;
	sonar_raw = x_new;
	x_post = x_pred +  0.91* (x_new - x_pred);//0.8461f
	v_post = v_pred +  6.2034f* (x_new - x_pred)*scale_kal_sonar_v;
  v_post=limit_mine(v_post,MAX_SPEED);
	  if(fabs(v_post) < 0.01)           \
    v_post = 0;   
	return x_pred;//m/s
}

// float t1r=1;
/* kalman filter states */
double x_pred_bmp = 0.0f; // m   0
double v_pred_bmp = 0.0f; //       1
double x_post_bmp = 0.0f; // m    2
double v_post_bmp = 0.0f; // m/s  3
float sonar_filter_bmp(float hight,float dt_sonar)
{float x_new;
 static float reg;
	float LPF_1=0.75; 
	float MAX_SPEED=0.5;
	/* no data for long time */
	if (dt_sonar > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred_bmp = x_post_bmp + dt_sonar * v_pred_bmp;
	v_pred_bmp = v_post_bmp;
   v_pred_bmp=limit_mine	(v_pred_bmp,MAX_SPEED);
	 if(fabs(v_pred_bmp) < 0.01)           \
    v_pred_bmp = 0;   
	 
	 v_pred_bmp=reg*(1-LPF_1)+v_pred_bmp*(LPF_1);
	 reg=v_pred_bmp;
	 x_new = hight;
	x_post_bmp = x_pred_bmp +  0.91* (x_new - x_pred_bmp);//0.8461f
	v_post_bmp = v_pred_bmp +  6.2034f* (x_new - x_pred_bmp)*scale_kal_sonar_v;
  v_post_bmp=limit_mine(v_post_bmp,MAX_SPEED);
	  if(fabs(v_post_bmp) < 0.01)           \
    v_post_bmp = 0;   
	return x_pred_bmp;//m/s
}

u8 state_dj[5]={0,0,0,0,0};
u8 state_dj_rx[5]={0,0,0,0,0};
u8 IO_STATE[5]={0,0,0,0,0};
u8 IO_STATER[5]={0,0,0,0,0};
u32 TEMP_SONAR=340*3000/200;//-------------------------------------超声波时间参数
//外部中断4服务程序
u32 cnt_sample1,now_dj[4],lastUpdate_dj[4];

void EXTI15_10_IRQHandler(void)
{     OSIntEnter();  
	float temp,temp1,temp2;
		static int ultra_distance_old;
	
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
				{
			IO_STATE[3]=GPIO_READ_SONAR(3);
			 switch(state_dj_rx[3])
			 {
				 case 0:if(IO_STATE[3]==1)
				 { 
					 lastUpdate_dj[3] =  GetSysTime_us();
					 state_dj_rx[3]=1; 
				 }
				 break;
				 case 1:		 
				 now_dj[3] = GetSysTime_us();  //读取时间
				 if( now_dj[3] <lastUpdate_dj[3]){cnt_sample1 =  ((float)( now_dj[3]  + (0xffff- lastUpdate_dj[3])) );}
				 else	{ cnt_sample1 =  ((float)( now_dj[3]  - lastUpdate_dj[3])); }
					 if(IO_STATE[3]==1)
						 state_dj_rx[3]=0; 
					 else if(IO_STATE[3]==0)
				 {  
				temp1=limit_mine(Roll,45);
				temp2=limit_mine(Pitch,45);
				temp=(float)cnt_sample1*1000/(TEMP_SONAR)*cos(temp1*0.017)*cos(temp2*0.017);
				temp=((temp)<(0)?(0):((temp)>(4500)?(4500):(temp)));
				ultra_distance=temp;//Moving_Median(1,5,temp);
				ultra_start_f = 0;
				state_dj_rx[3]=0;
		    sys.sonar=ultra_ok = 1;
				T_sonar=Get_Cycle_T(GET_T_SONAR_SAMPLE);
				//ultra_delta = (ultra_distance - ultra_distance_old)/LIMIT(T_sonar,0.000000001,1);
				//sonar_filter((float) temp/1000,T_sonar);
				//ultra_distance_old = ultra_distance;
				 }
				
				 break;
				 default:
					 state_dj_rx[3]=0;
					 break;
			 }
				
					EXTI_ClearITPendingBit(EXTI_Line11);
	}   
				   OSIntExit();
}


int ultra_distance,ultra_distance_r;
float ultra_delta;
u16 laser_sonar_buf[20];
void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp;
	int temp1,temp2,temp;
	float dt;
	static int ultra_distance_old;
	if( ultra_start_f == 1 )
	{
		ultra_tmp = com_data;
		ultra_start_f = 2;
	}
	else if( ultra_start_f == 2 )
	{ module.sonar=1;
		temp = (ultra_tmp<<8) + com_data;
		ultra_start_f = 0; 
		dt=Get_Cycle_T(GET_T_SONAR_SAMPLE);
		temp1=limit_mine(Roll,45);
		temp2=limit_mine(Pitch,45);
		temp=temp*cos(temp1*0.017)*cos(temp2*0.017);
		if(temp<3200){
	  temp=((temp)<(0)?(0):((temp)>(4500)?(4500):(temp)));
   	//sonar_filter((float) temp/1000.,dt);
	  //ultra_distance=((x_pred*1000)<(0)?(0):((x_pred*1000)>(2500)?(2500):(x_pred*1000)));
			//if(height_ctrl_mode==1)
			//ultra_distance=baroAlt*10;
			//else
		
		 T_sonar=Get_Cycle_T(GET_T_SONAR_SAMPLE); 
		ultra_distance=temp;//Moving_Median(1,5,temp);
		}
		sys.sonar=ultra_ok = 1;
	}
	
	 static u8 state,cnt,buf[8];
	 u8 sum;
	 switch(state){
		 case 0:
			  if(com_data==0x55)
		       state=1;
		 break;
	   case 1:
			  if(com_data==0xAA)
				{state=2;cnt=0;}
				else
					 state=0;
		 break;
	   case 2:
			  buf[cnt++]=com_data;
		    if(cnt>6)
				{
				sum=0x55+0xAA+buf[0]+buf[1]+buf[2]+buf[3]+buf[4];
				   if(sum==buf[5])	
					 {
					 module.sonar=sys.sonar=ultra_ok = 1;
						T_sonar=Get_Cycle_T(GET_T_SONAR_SAMPLE); 
						ultra_distance= LIMIT((buf[3]<<8|buf[4])*10,0,7500);
					 }
        state=0;				 
				}
		 break;
	 }	 
	
	 
	static u8 state_laser;
  static u8 cnt_laser;
	u8 check,i;
  u16 Strength,dis;
	
	switch(state_laser)
	{
		case 0:
			if(com_data==0x59)
			{state_laser=1;laser_sonar_buf[cnt_laser++]=com_data;}
		break;
		case 1:
			if(com_data==0x59)
			{state_laser=2;laser_sonar_buf[cnt_laser++]=com_data;}
			else 
			 state_laser=0;
		break	;
		case 2:
			 if(cnt_laser<=8)
			  laser_sonar_buf[cnt_laser++]=com_data;
			 else
			 {state_laser=0;
				cnt_laser=0;
				 check=laser_sonar_buf[0]+laser_sonar_buf[1]+laser_sonar_buf[2]+laser_sonar_buf[3]
				 +laser_sonar_buf[4]+laser_sonar_buf[5]+laser_sonar_buf[6]+laser_sonar_buf[7];
				 if(check==laser_sonar_buf[8])
				 {
				
				 dis=(laser_sonar_buf[3]<<8|laser_sonar_buf[2]); 
				 Strength=laser_sonar_buf[5]<<8|laser_sonar_buf[4];
			    if(Strength>20){
					   module.sonar=sys.sonar=ultra_ok = 1;
						T_sonar=Get_Cycle_T(GET_T_SONAR_SAMPLE); 
						ultra_distance= LIMIT(dis*10,0,7500);
					}
				 }
				 for(i=0;i<9;i++)
				  laser_sonar_buf[i]=0;
			 }
			 break;
	}

}


