

#include "pwm_out.h"
#include "include.h"
#include "my_math.h"

//21·ÖÆµµ½ 84000000/21 = 4M   0.25us

#define INIT_DUTY 4000 //u16(1000/0.25)
#define ACCURACY 10000 //u16(2500/0.25) //accuracy
#define PWM_RADIO 4//(8000 - 4000)/1000.0
void 	MOTOR_SET(void)
{
	
Delay_ms(4000);
 	TIM1->CCR1 = PWM_RADIO *( 1000) + INIT_DUTY;				//5	
 	TIM1->CCR2 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//6	
  TIM1->CCR3 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM1->CCR4 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
Delay_ms(4000);
	TIM1->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//5	
 	TIM1->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//6	
  TIM1->CCR3 = PWM_RADIO *(0 ) + INIT_DUTY;				//7	
  TIM1->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	
}
u8 PWM_Out_Init(uint16_t hz)//400hz
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t PrescalerValue = 0;
	u32 hz_set = ACCURACY*hz;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	
	hz_set = LIMIT (hz_set,1,84000000);
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOE, ENABLE);
////////////////////////////////////////////////////////////////////////////////////

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
	//------------------
	hz_set=50;//50hz
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 40000;//ACCURACY;									
  TIM_TimeBaseStructure.TIM_Prescaler = 83;//PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

 
	
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);

	
  Set_DJ(0,0,0,0);
	if( hz_set > 84000000 )
	{
		return 0;
	}
	else
	{
		return 1;
	}
	
	
}
u8 PMW_T=1;
u8 CH_out_Mapping[MAXMOTORS] = {0,1,2,3};

void CH_out_Mapping_Fun(u16 *out,u16 *mapped )
{
	u8 i;
	for( i = 0 ; i < MAXMOTORS ; i++ )
	{
		*( mapped + i ) = *( out + CH_out_Mapping[i] );
	}
}
u8 sel=0;
void SetPwm(int16_t pwm[MAXMOTORS],s16 min,s16 max)
{
	u8 i;
	s16 pwm_tem[MAXMOTORS];

	for(i=0;i<MAXMOTORS;i++)
	{
			pwm_tem[i] = pwm[i] ;
			pwm_tem[i] = LIMIT(pwm_tem[i],min,max);
	}
	
//	#if NEW_FLY_BOARD
	//TIM1->CCR4 = PWM_RADIO *( pwm_tem[0] ) + INIT_DUTY;				//1	
//	TIM1->CCR2 =0; PWM_RADIO *( pwm_tem[2] ) + INIT_DUTY;				//2

//	#else
// 	TIM5->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[1]] ) + INIT_DUTY;				//5	
// 	TIM5->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[0]] ) + INIT_DUTY;				//6	
//  TIM8->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[2]] ) + INIT_DUTY;				//7	
//  TIM8->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[3]] ) + INIT_DUTY;				//8	
//	#endif
}
#define PWM_PER_DEGREE 2000/180
float TEMP_DJ=22.22;
float PWM_dj[4]={0};
float ang_off[2]={0,0};
float PWM_dj1;
#define ANG_DJ 25
void Set_DJ(float ang1,float ang2,float ang3,float ang4)
{

 // PWM_dj1=0+TEMP_DJ*LIMIT(-ang1+0,-ANG_DJ,ANG_DJ);
	//PWM_dj[1]=0+TEMP_DJ*LIMIT(-ang2+0,-ANG_DJ,ANG_DJ);
	TIM8->CCR3 =PWM_dj1;// PWM[0];
	TIM8->CCR4 =PWM_dj1;
	//TIM8->CCR4 =PWM_dj[1]+3800-ang_off[1];// PWM[1];
//	TIM1->CCR3 = 500+PWM[2];
//	TIM1->CCR2 = 500+PWM[3];
	

}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
