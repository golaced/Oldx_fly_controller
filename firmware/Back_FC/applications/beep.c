#include   "beep.h"
#include   "time.h"
#include   "bat.h"
#include   "include.h"
#include 	 "rc.h"
#include 	 "led.h"
u8 music_sel;
//  11  21
//音阶(0~3)音高(0~7)     循环次数(0~7)  休止符长度(0~7)
#define B1 13
#define B2 16
#define B3 15
u8 start_music_px4[]={
 B1,12, B2,12, B3,12,  B1,12, B2,12, B3,12,  B1,12, B2,12, B3,12,   B1,12, B2,12, B3,12,  B2,12,  B3,11,  B2,11, B3,11,  B2,11, B3,43,
};


u8 start_music_windows[]={
 21,21, 01,11, 21,21, 01,11, 21,21,01,11,      11,13,  15,13,    13,11, 01,11, 13,11, 01,11, 13,11, 01,11, 
	21,23,   15,41, 01,11, 15,31, 01,11, 15,21, 01,11,  15,11, 01,11,
};

u8 start_music_micro[]={
 21,21, 01,11, 21,21, 01,11, 21,21,01,11,      11,13,  15,13,    13,11, 01,11, 13,11, 01,11, 13,11, 01,11, 
	21,23,   15,41, 01,11, 15,31, 01,11, 15,21, 01,11,  15,11, 01,11,
};

u8 mems_gps_music[]={
 15,44,  17,33, 17,33
};

u8 mems_gps_loss[]={
 15,64,   13,44
};


u8 mems_right_music[]={
 05,64,  00,13, 12,64
};

u8 mems_error_music[]={
 02,34, 00,32, 02,34, 00,32, 
};

u8 mission_music[]={
 22,35, 22,35, 22,65,  
};

u8 mems_wayup_music[]={
 15,52,  17,32, 17,32,
};

u8 bat_error_music[]={
 07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  
 07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32, 	
};

u8 rc_error_music[]={
 27,11, 00,11,   27,11, 00,11,  27,11, 00,11, 00,39 , 00,39
};

u8 hml_cal_music[]={
 13,11, 00,11,   13,11, 00,11,  13,11, 00,11, 00,39 , 00,39
};


u8 start_music_pix[]={
 04,32, 24,22, 05,32, 
 04,32, 24,22, 05,32,
 04,32, 24,22, 05,32,
};


u8 fc_state_beep[3];
u8 beep_state[]={
 00,11, 00,21,   00,11, 00,21,  00,11, 00,21, 00,11, 00,11,
};

u8 fc_save_gps_beep;
u8 beep_gps_save[]={
 00,11, 00,21,   00,11, 00,21,  
};

u8 test_to24;
u16 Beat_delay[7]={0,62,94,125,125,187,250};
void Play_Music(u8 *music, u16 st,u16 ed)
{
	static u8 flag;
	u16 i;
  for(i=st;i<ed-st;i++)
   {
		 u8 level=music[i*2]/10;
		 u8 tone=music[i*2]%10;
		 Tone(level,tone);
		 u8 loop=music[i*2+1]%10;
		 u8 beat=music[i*2+1]/10;
     Delay_ms(Beat_delay[beat]*loop);	 
		 if(music_sel==START_BEEP){
			 LED_ON(flag);
		   flag=!flag;
		 }
	 }	
   Tone(0,0);		 
}

u8 task_play_flag[2];
u8 Play_Music_In_Task(u8 *music, u16 st,u16 ed,u8 en,float dt)
{ static u8 state=0;
	static u16 i,cnt;
	
	switch(state)
	{
		case 0:
	     if(en)
	     {state=1;i=st;cnt=0;}
	  break;
	  case 1:
			 if(en) 
			 {
				u8 level=music[i*2]/10;
				u8 tone=music[i*2]%10;
				u8 loop=music[i*2+1]%10;
				u8 beat=music[i*2+1]/10;
				 
				 if(cnt++>Beat_delay[beat]*loop/1000./dt){Tone(level,tone);i++;cnt=0;
				 				} 
			 }
	     else
			 {Tone(0,0);state=0;  }
			 if(i>ed)
			 {state=0;Tone(0,0);task_play_flag[0]=0;	}
		break;
	} 
}

void Play_Music_Direct(u8 sel)
{
	music_sel=0;
//	#if USE_VER_8_PWM==0
  switch(sel)
	{
		case MEMS_RIGHT_BEEP:
	  Play_Music(mems_right_music,0,sizeof(mems_right_music)/2);	
	  break;
		case START_BEEP:
		music_sel=	START_BEEP;
		Play_Music(start_music_px4,0,sizeof(start_music_px4)/2);	
		break;
		case MEMS_ERROR_BEEP:
		Play_Music(mems_error_music,0,sizeof(mems_error_music)/2);		
		break;
		case MEMS_WAY_UPDATE:
		Play_Music(mems_wayup_music,0,sizeof(mems_wayup_music)/2);		
		break;
		case MEMS_GPS_RIGHT:
		Play_Music(mems_gps_music,0,sizeof(mems_gps_music)/2);		
		break;
		case MEMS_GPS_LOSS:
		Play_Music(mems_gps_loss,0,sizeof(mems_gps_loss)/2);		
		break;
	}
 // #endif
}

void Play_Music_Task(u8 sel,float dt)//<-----------------
{
	//Tone(0,0);
	if(NS==2)
  switch(sel)
	{
		case MEMS_RIGHT_BEEP:
	  Play_Music_In_Task(mems_right_music,0,sizeof(mems_right_music)/2,1,dt);	
	  break;
		case START_BEEP:
		Play_Music_In_Task(start_music_px4,0,sizeof(start_music_px4)/2,1,dt);	
		break;
		case MEMS_GPS_RIGHT:
		Play_Music_In_Task(mems_gps_music,0,sizeof(mems_gps_music)/2,1,dt);		
		break;
		case MEMS_GPS_LOSS:
		Play_Music_In_Task(mems_gps_loss,0,sizeof(mems_gps_loss)/2,1,dt);		
		break;
		case MEMS_ERROR_BEEP:
		Play_Music_In_Task(mems_error_music,0,sizeof(mems_error_music)/2,1,dt);		
		break;
		case BEEP_MISSION:
		Play_Music_In_Task(mission_music,0,sizeof(mission_music)/2,1,dt);			
		break;
		case BAT_ERO_BEEP:
		#if defined(DEBUG_MODE)
    Tone(0,0);
		#else
		Play_Music_In_Task(bat_error_music,0,sizeof(bat_error_music)/2,1,dt);		
		#endif
		break;
		case RC_ERO_BEEP:
		;//Play_Music_In_Task(rc_error_music,0,sizeof(rc_error_music)/2,1,dt);		
		break;
		case BEEP_STATE:
		beep_state[0]=fc_state_beep[0];	
		beep_state[2]=fc_state_beep[1];	
		beep_state[4]=fc_state_beep[2];	
		#if defined(DEBUG_MODE)
    Tone(0,0);
		#else
		Play_Music_In_Task(beep_state,0,sizeof(beep_state)/2,1,dt);		
		#endif
		break;
		case BEEP_GPS_SAVE:
		beep_gps_save[0]=beep_gps_save[2]=fc_save_gps_beep;	
		Play_Music_In_Task(beep_gps_save,0,sizeof(beep_gps_save)/2,1,dt);		
		break;
		case BEEP_HML_CAL:
		Play_Music_In_Task(hml_cal_music,0,sizeof(hml_cal_music)/2,1,dt);		
		break;
		
		default:
			Tone(0,0);
		break;
	}
}

void Beep_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
//	#if !USE_VER_8
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM14时钟使能    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTF时钟	
//	
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8); //GPIOF9复用为定时器14
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
//	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PF9
//	  
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//初始化定时器14
//	
//	//初始化TIM14 Channel1 PWM模式	 
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = arr/2;
//	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

//	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
// 
//  TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE使能 
//	
//	TIM_Cmd(TIM8, ENABLE);  //使能TIM14
//  Tone(0,0);
//		
//		
//	Play_Music_Direct(START_BEEP);
////  u8 i,j;
////	for(i=0;i<3;i++)
////	 for(j=1;j<8;j++)
////	 {Tone(i,j);Delay_ms(111);}
////	 Tone(0,0);
//  #else
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2); //GPIOF9复用为定时器14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/2;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIM14
  Tone(0,0);
		
		
//	Play_Music_Direct(START_BEEP);
//  u8 i,j;
//	for(i=0;i<3;i++)
//	 for(j=1;j<8;j++)
//	 {Tone(i,j);Delay_ms(111);}
//	 Tone(0,0);
 // #endif
}  

u16 tone_table[3][8]={
     {0,261,293,329,349,391,440,493},
     {0,523,587,659,698,783,880,987},
     {0,1046,1174,1318,1396,1567,1760,1975}};
void Tone(u8 level, u8 tone)
{
//	#if !USE_VER_8&&!USE_VER_8_PWM
//	u32 psc=84-1;
//  u32 arr=1000000/tone_table[level][tone]-1;
//  if(tone==0)
//		arr=1000000/1-1;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//初始化定时器14
//	
//	//初始化TIM14 Channel1 PWM模式	 
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = arr/2;
//	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

//	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
// 
//  TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE使能 
//	
//	TIM_Cmd(TIM8, ENABLE);  //使能TIM14
////	TIM8->PSC=psc;
////	TIM8->ARR=arr;
////	TIM8->CCR4=arr/2;
//  #else
	u32 psc=84/2-1;
  u32 arr=1000000/tone_table[level][tone]-1;
  if(tone==0)
		arr=1000000/1-1;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/2;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIM14
//	TIM8->PSC=psc;
//	TIM8->ARR=arr;
//	TIM8->CCR4=arr/2;
 // #endif
} 
