
#include "LIS3MDL.h"
#include "led_fc.h"
#include "include.h"
#include "mpu6050.h"
#include "hml5833l.h"
#include "usart_fc.h"
void LEDRGB_COLOR(u8 color);
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
#if USE_MINI_BOARD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
#else
	RCC_AHB1PeriphClockCmd(ANO_RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED1| ANO_Pin_LED2| ANO_Pin_LED3| ANO_Pin_LED4;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);
#endif	
	LEDRGB_COLOR(BLUE);
	Delay_ms(500);
	LEDRGB_COLOR(RED);
	Delay_ms(500);
	LEDRGB_COLOR(GREEN);
  Delay_ms(500);



	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}



/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/


void LEDRGB(u8 sel,u8 on)
{
switch(sel)
{
#if USE_MINI_BOARD
case BLUE:
if(!on)
GPIO_ResetBits(GPIOC,GPIO_Pin_2);
else
GPIO_SetBits(GPIOC,GPIO_Pin_2);
break;
case RED:
if(!on)
GPIO_ResetBits(GPIOC,GPIO_Pin_3);
else
GPIO_SetBits(GPIOC,GPIO_Pin_3);
break;
case  GREEN:
if(!on)
GPIO_ResetBits(GPIOC,GPIO_Pin_4);
else
GPIO_SetBits(GPIOC,GPIO_Pin_4);
break;
#else	
case BLUE:
if(!on)
GPIO_ResetBits(GPIOE,GPIO_Pin_0);
else
GPIO_SetBits(GPIOE,GPIO_Pin_0);
break;
case RED:
if(!on)
GPIO_ResetBits(GPIOE,GPIO_Pin_1);
else
GPIO_SetBits(GPIOE,GPIO_Pin_1);
break;
case  GREEN:
if(!on)
GPIO_ResetBits(GPIOE,GPIO_Pin_2);
else
GPIO_SetBits(GPIOE,GPIO_Pin_2);
break;
#endif
}
}

void LEDRGB_COLOR(u8 color)
{
switch (color)
{
case RED:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case BLUE:
LEDRGB(RED,0);
LEDRGB(BLUE,1);
LEDRGB(GREEN,0);
break;
case GREEN:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
case WHITE:
LEDRGB(RED,1);
LEDRGB(BLUE,1);
LEDRGB(GREEN,1);
break;
case BLACK:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case YELLOW:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
}
}
#define IDLE 0
#define CAL_MPU 1
#define CAL_M  2
void LEDRGB_STATE(void)
{
static u8 main_state;
static u8 mpu_state,m_state,idle_state;
static u16 cnt,cnt_idle;
u8 mode_control;
	
	
mode_control=mode.mode_fly;

if(lis3mdl.Gyro_CALIBRATE||lis3mdl.Acc_CALIBRATE)
{idle_state=0;mpu_state=main_state=CAL_MPU;

}
else if(lis3mdl.Mag_CALIBRATED)
{idle_state=0;mpu_state=main_state=CAL_M;
}
else
main_state=0;	

//   | | | |    | | | |   | | | |   | | | |   | | | |
//    ARM          GPS1     GPS2      GPS3      MODE  
#define RGB_DELAY 3
switch(idle_state)
{//ARM
	case 0:
		if(main_state==IDLE)
			{idle_state=1;cnt_idle=0;}
	break;
	case 1:
	 if(lis3mdl.Cali_3d)
				LEDRGB_COLOR(GREEN); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY*2)
	{idle_state=2;cnt_idle=0;}
	break;
	case 2:
		LEDRGB_COLOR(BLACK);	
	if(cnt_idle++>RGB_DELAY)
	{idle_state=3;cnt_idle=0;}
	break;
	case 3:
			 if(lis3mdl.Cali_3d)
				LEDRGB_COLOR(GREEN); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=4;cnt_idle=0;}
	break;
//GPS1	
	case 4:
		if(module.acc==2)//m100.connect&&m100.m100_data_refresh&&m100.GPS_STATUS>=3)
			LEDRGB_COLOR(WHITE);
		else if(module.acc==1)
			LEDRGB_COLOR(BLUE);
		else
			LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=5;cnt_idle=0;}
	break;
	case 5:
	  if(module.acc==2)//m100.connect&&m100.m100_data_refresh&&m100.GPS_STATUS>=3)
			LEDRGB_COLOR(WHITE);
		else if(module.acc==1)
			LEDRGB_COLOR(BLUE);
		else
			LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=6;cnt_idle=0;}
	break;
	case 6:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=7;cnt_idle=0;}
	break;
	case 7:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=8;cnt_idle=0;}
	break;
		
//GPS2
	case 8:
	  if(module.gyro==2)//m100.connect&&m100.m100_data_refresh&&m100.GPS_STATUS>=3)
			LEDRGB_COLOR(WHITE);
		else if(module.gyro==1)
			LEDRGB_COLOR(BLUE);
		else
			LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=9;cnt_idle=0;}
	break;
	case 9:
	  if(module.gyro==2)//m100.connect&&m100.m100_data_refresh&&m100.GPS_STATUS>=3)
			LEDRGB_COLOR(WHITE);
		else if(module.gyro==1)
			LEDRGB_COLOR(BLUE);
		else
			LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=10;cnt_idle=0;}
	break;
	case 10:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=11;cnt_idle=0;}
	break;
	case 11:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=12;cnt_idle=0;}
	break;
//GPS3	
	case 12:
	  if(module.hml==2)//m100.connect&&m100.m100_data_refresh&&m100.GPS_STATUS>=3)
			LEDRGB_COLOR(WHITE);
		else if(module.hml==1)
			LEDRGB_COLOR(BLUE);
		else
			LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=13;cnt_idle=0;}
	break;
	case 13:
	  if(module.hml==2)//m100.connect&&m100.m100_data_refresh&&m100.GPS_STATUS>=3)
			LEDRGB_COLOR(WHITE);
		else if(module.hml==1)
			LEDRGB_COLOR(BLUE);
		else
			LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=14;cnt_idle=0;}
	break;
	case 14:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=15;cnt_idle=0;}
	break;
	case 15:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=0;cnt_idle=0;}
	break;
//MODE
	case 16:
		switch(qr.use_spd)
	   {
			 case 1:	LEDRGB_COLOR(YELLOW);break;//zit
			 case 0:  LEDRGB_COLOR(BLUE);break;//gps
		 }
		
	if(cnt_idle++>RGB_DELAY)
	{idle_state=17;cnt_idle=0;}
	break;
	case 17:
			switch(qr.use_spd)
	   {
			 case 1:	LEDRGB_COLOR(YELLOW);break;//zit
			 case 0:  LEDRGB_COLOR(BLUE);break;//gps
		 }
	if(cnt_idle++>RGB_DELAY)
	{idle_state=18;cnt_idle=0;}
	break;
	case 18:
			switch(qr.use_spd)
	   {
			 case 1:	LEDRGB_COLOR(BLACK);break;//zit
			 case 0:  LEDRGB_COLOR(BLACK);break;//gps
		 }
	if(cnt_idle++>RGB_DELAY)
	{idle_state=19;cnt_idle=0;}
	break;
	case 19:
				switch(mode_control)
	   {
			 case 0:	LEDRGB_COLOR(BLACK);break;//zit
			 case 1:  LEDRGB_COLOR(BLACK);break;//gps
		 }
	if(cnt_idle++>RGB_DELAY)
	{idle_state=20;cnt_idle=0;}
	break;
//-END
	case 20:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>20)
	{idle_state=0;cnt_idle=0;}
	break;
}

switch(main_state){
	case CAL_MPU:
			mpu_state=3;LEDRGB_COLOR(YELLOW);
		    break;
	case CAL_M:
			mpu_state=3;LEDRGB_COLOR(BLUE);
	     break;
	case 3:
		 if(cnt++>10)
		 {cnt=0;
		 if(mpu_state==CAL_MPU)
		 main_state=CAL_MPU;
		 else
		 main_state=CAL_M;	 
		 }
		 break;
	 }	 


}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

