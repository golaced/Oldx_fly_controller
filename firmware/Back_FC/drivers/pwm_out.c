

#include "pwm_out.h"
#include "include.h"
#include "mymath.h"
#include "ctrl.h"
//21分频到 84000000/21 = 4M   0.25us

#define INIT_DUTY 4000 //u16(1000/0.25)
#define ACCURACY 10000 //u16(2500/0.25) //accuracy
#define PWM_RADIO 4//(8000 - 4000)/1000.0
u8 motor_cal=0;//电调校准标志位
void 	MOTOR_SET(void)
{
#if USE_MINI_BOARD
Delay_ms(1000);
 	TIM3->CCR1 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//5	
 	TIM3->CCR2 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//6	
  TIM3->CCR3 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM3->CCR4 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
	TIM4->CCR1 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM4->CCR2 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
	TIM4->CCR3 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM4->CCR4 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
Delay_ms(2000);
	TIM3->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//5	
 	TIM3->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//6	
  TIM3->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
  TIM3->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	TIM4->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
  TIM4->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	TIM4->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
  TIM4->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
Delay_ms(100);	
#else	
Delay_ms(4000);
 	TIM1->CCR1 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//5	
 	TIM1->CCR2 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//6	
  TIM1->CCR3 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM1->CCR4 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
	TIM4->CCR1 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//7	
  TIM4->CCR2 = PWM_RADIO *( 1000 ) + INIT_DUTY;				//8	
Delay_ms(4000);
	TIM1->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//5	
 	TIM1->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//6	
  TIM1->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
  TIM1->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	TIM4->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
  TIM4->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
#endif
	
}
u8 PWM_Out_Init(uint16_t hz)//400hz
{
	#if USE_MINI_BOARD
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		uint16_t PrescalerValue = 0;
		u32 hz_set = ACCURACY*hz*2;
    #if USE_FAN_AS_BLDC
	  hz_set = ACCURACY*200*2;
		#endif
		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_OCStructInit(&TIM_OCInitStructure);

		hz_set = LIMIT (hz_set,1,84000000);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOB, ENABLE);

		//////////////////////////////////TIM3///////////////////////////////
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3); 
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

	/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

		 /* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel2 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		//TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel3 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		//TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel4 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC4Init(TIM3, &TIM_OCInitStructure);
		//TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
		
		TIM_CtrlPWMOutputs(TIM3, ENABLE);
		TIM_ARRPreloadConfig(TIM3, ENABLE);
		TIM_Cmd(TIM3, ENABLE);	

		//////////////////////////////////////TIM4///////////////////////////////////////////
	  hz_set = ACCURACY*hz;
		hz_set = LIMIT (hz_set,1,84000000);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_8 | GPIO_Pin_9; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;//GPIO_PuPd_UP ;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
		//------------------
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel2 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		//TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);

		TIM_CtrlPWMOutputs(TIM4, ENABLE);
		TIM_ARRPreloadConfig(TIM4, ENABLE);
		TIM_Cmd(TIM4, ENABLE);	

		if(motor_cal)//《---------------在此处加断点 校准电调
		MOTOR_SET();
		#if USE_FAN_AS_BLDC
		TIM3->CCR1 = fan.off[0];				//5	
		TIM3->CCR2 = fan.off[1];				//6	
		TIM3->CCR3 = fan.off[2];				//7	
		TIM3->CCR4 = fan.off[3];				//8	
		#else
		TIM3->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//5	
		TIM3->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//6	
		TIM3->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM3->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
		#endif
		TIM4->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM4->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
		TIM4->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM4->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
		#if MAXMOTORS==12
	  TIM1->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM8->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	  TIM8->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM8->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	  #endif
		if( hz_set > 84000000 )
		{
		return 0;
		}
		else
		{
		return 1;
		}
	
	
	#else
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		uint16_t PrescalerValue = 0;
		u32 hz_set = ACCURACY*hz;

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_OCStructInit(&TIM_OCInitStructure);
		
		hz_set = LIMIT (hz_set,1,84000000);
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOB, ENABLE);

	//////////////////////////////////TIM1///////////////////////////////
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
		GPIO_Init(GPIOE, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
		
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

		 /* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);
		//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel2 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);
		//TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel3 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC3Init(TIM1, &TIM_OCInitStructure);
		//TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel4 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC4Init(TIM1, &TIM_OCInitStructure);
		//TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
		
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		TIM_ARRPreloadConfig(TIM1, ENABLE);
		TIM_Cmd(TIM1, ENABLE);	
			
		//////////////////////////////////////TIM4///////////////////////////////////////////

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;//GPIO_PuPd_UP ;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
		//------------------
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		 /* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel2 */
		//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		//TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		
		TIM_CtrlPWMOutputs(TIM4, ENABLE);
		TIM_ARRPreloadConfig(TIM4, ENABLE);
		TIM_Cmd(TIM4, ENABLE);	

	if(motor_cal)//《---------------在此处加断点
		MOTOR_SET();
		TIM1->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//1
		TIM1->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//2	
		TIM1->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//3	
		TIM1->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//4	
		TIM4->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//5	
		TIM4->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//6	
    TIM4->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM4->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	  #if MAXMOTORS==12
	  TIM1->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM8->CCR1 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	  TIM8->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//7	
		TIM8->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//8	
	  #endif
		if( hz_set > 84000000 )
		{
			return 0;
		}
		else
		{
			return 1;
		}
	#endif
	
}

u8 PWM_AUX_Out_Init(uint16_t hz)//50Hz
{
	
	
	#if USE_VER_6||USE_VER_7
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		uint16_t PrescalerValue = 0;
		u32 hz_set = ACCURACY*hz*2;

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_OCStructInit(&TIM_OCInitStructure);

		hz_set = LIMIT (hz_set,1,84000000);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	
		//////////////////////////////////////TIM8///////////////////////////////////////////
	  hz_set = ACCURACY*hz;
		hz_set = LIMIT (hz_set,1,84000000)/2;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;//GPIO_PuPd_UP ;
		GPIO_Init(GPIOA, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);
		//------------------
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC4Init(TIM1, &TIM_OCInitStructure);


		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		TIM_ARRPreloadConfig(TIM1, ENABLE);
		TIM_Cmd(TIM1, ENABLE);	
	#else
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		uint16_t PrescalerValue = 0;
		u32 hz_set = ACCURACY*hz*2;
  #endif	
	#if USE_VER_8||USE_VER_8_PWM
		PrescalerValue = 0;
		hz_set = ACCURACY*hz*2;
	#endif
		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_OCStructInit(&TIM_OCInitStructure);

		hz_set = LIMIT (hz_set,1,84000000);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	
		//////////////////////////////////////TIM8///////////////////////////////////////////
	  hz_set = ACCURACY*hz;
		hz_set = LIMIT (hz_set,1,84000000)/2;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_8; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;//GPIO_PuPd_UP ;
		GPIO_Init(GPIOC, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
		//------------------
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM8, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM8, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC3Init(TIM8, &TIM_OCInitStructure);

		TIM_CtrlPWMOutputs(TIM8, ENABLE);
		TIM_ARRPreloadConfig(TIM8, ENABLE);
		TIM_Cmd(TIM8, ENABLE);	
		#if MAXMOTORS!=12
    SetPwm_AUX(0,0);
		aux34(0,0);  	
		#endif
}

AUX_S aux;
void SetPwm_AUX(float pit,float rol)
{ static u8 init;
	u8 i;
	if(!init)
	{
	init=1;
	#if FAN_IS_3AXIS
	aux.init[0]=1500;
	aux.init[1]=1500;
	aux.min[0]=1000;
	aux.min[1]=1000;
  aux.max[0]=2000;
	aux.max[1]=2000;
  aux.att_off[0]=0;	
	aux.flag[0]=-1;	
  aux.flag[1]=1;		
	aux.pwm_per_dig[0]=3;
	aux.pwm_per_dig[1]=4.9;	
  #else	
	aux.init[0]=1980;
	aux.init[1]=1625;
	aux.min[0]=500;
	aux.min[1]=1400;
  aux.max[0]=2500;
	aux.max[1]=1780;
  
	#if defined(INDOOR_FPV_SERCH)
  aux.att_off[0]=-22;	
	#else
	aux.att_off[0]=-46;	
	#endif
	if(mcuID[0]==0x2B&&mcuID[1]==0x17&&mcuID[2]==0x31)	
	aux.flag[0]=-1;
	else
	aux.flag[0]=1;	
  aux.flag[1]=1;		
	aux.pwm_per_dig[0]=aux.pwm_per_dig[1]=4.8;
  #endif	
	
	#if USE_VER_7
  aux.init[0]=1910;
	aux.init[1]=1650;
	aux.flag[0]=1;	
	aux.flag[1]=1;		
	#endif
	#if USE_VER_8
  aux.init[0]=1500;
	aux.init[1]=1500;
	aux.flag[0]=-1;	
	aux.flag[1]=1;		
	aux.att_off[0]=-10;	
	aux.att_off[1]=0;	
	aux.pwm_per_dig[0]=6.3;//pitch
	aux.pwm_per_dig[1]=3;	//yaw
	aux.min[0]=500;
	aux.min[1]=500;
  aux.max[0]=2500;
	aux.max[1]=2500;
	#endif
	#if defined(M_DRONE_380X4S)
		#if defined(BLDC_PAN)
		aux.att_off[0]=0;	
		aux.init[0]=1500;
		aux.init[1]=1550;
		aux.flag[0]=-1;	
		aux.flag[1]=1;
		aux.min[0]=550;
		aux.min[1]=550;
		aux.max[0]=2500;
		aux.max[1]=1900;
		aux.pwm_per_dig[0]=5.5;
		aux.pwm_per_dig[1]=4.9;	
		#else
		aux.att_off[0]=45;	
		aux.init[0]=1430;
		aux.init[1]=1550;
		aux.flag[0]=1;	
		aux.flag[1]=1;
		aux.min[0]=500;
		aux.min[1]=600;
		aux.max[0]=2500;
		aux.max[1]=1900;
		aux.pwm_per_dig[0]=6.6;
		aux.pwm_per_dig[1]=4.9;
		#endif
	#endif
	#if defined(M_DRONE_330X6)
		#if defined(BLDC_PAN)
		aux.att_off[0]=0;	
		aux.init[0]=1500;
		aux.init[1]=1550;
		aux.flag[0]=-1;	
		aux.flag[1]=1;
		aux.min[0]=1252;
		aux.min[1]=600;
		aux.max[0]=2500;
		aux.max[1]=1900;
		aux.pwm_per_dig[0]=5.5;
		aux.pwm_per_dig[1]=4.9;	
		#else
		aux.att_off[0]=0;	
		aux.init[0]=1580;
		aux.init[1]=1620;
		aux.flag[0]=1;	
		aux.flag[1]=1;
		aux.min[0]=500;
		aux.min[1]=600;
		aux.max[0]=2500;
		aux.max[1]=1760;
		aux.pwm_per_dig[0]=4.78;
		aux.pwm_per_dig[1]=4.9;
		#endif
	#endif
	}	
	aux.pwm_tem[0]=aux.init[0]+aux.pwm_per_dig[0]*pit*aux.flag[0];
	aux.pwm_tem[1]=aux.init[1]+aux.pwm_per_dig[1]*rol*aux.flag[1];
	for(i=0;i<2;i++)
	{
			aux.pwm_tem[i] = LIMIT(aux.pwm_tem[i],aux.min[i],aux.max[i]);
	}
	#if MAXMOTORS!=12
		#if USE_VER_8||USE_VER_8_PWM
		TIM1->CCR1 = (aux.pwm_tem[0] )/2 ;				//1	
		TIM8->CCR1 = (aux.pwm_tem[1] )/2 ;				//2
		#else
		#if USE_VER_6||USE_VER_7
		TIM1->CCR1 = (aux.pwm_tem[0] )/2 ;				//1	
		TIM1->CCR4 = (aux.pwm_tem[1] )/2 ;				//2
		#else
		TIM8->CCR1 = (aux.pwm_tem[0] )/2 ;				//1	
		TIM8->CCR2 = (aux.pwm_tem[1] )/2 ;				//2
		#endif
		#endif
	#endif
}	

void aux34(int c3,int c4)
{
	static u8 init;
	if(!init){init=1;
	aux.init[2]=1500;
	aux.flag[2]=-1;			
	aux.att_off[2]=0;	
	aux.pwm_per_dig[2]=6.3;//pitch
	aux.min[2]=500;
	aux.max[2]=2500;
	aux.init[3]=1500;
	aux.flag[3]=-1;			
	aux.att_off[3]=0;	
	aux.pwm_per_dig[3]=6.3;//pitch
	aux.min[3]=500;
	aux.max[3]=2500;
	}
	aux.pwm_tem[2]=aux.init[2]+aux.pwm_per_dig[2]*c3*aux.flag[2];
	aux.pwm_tem[3]=aux.init[3]+aux.pwm_per_dig[3]*c4*aux.flag[3];
	aux.pwm_tem[2] = LIMIT(aux.pwm_tem[2],aux.min[2],aux.max[2]);
  aux.pwm_tem[3] = LIMIT(aux.pwm_tem[3],aux.min[3],aux.max[3]);
  #if MAXMOTORS!=12
	TIM8->CCR2 = (aux.pwm_tem[2] )/2 ;				//2
	TIM8->CCR3 = (aux.pwm_tem[3] )/2 ;				//2
	#endif
}	
u16 BEEP_RATE;
u8 PMW_T=1;
#if USE_MINI_BOARD
u8 CH_out_Mapping[8] = {0,1,2,3,4,5,6,7};
#else
u8 CH_out_Mapping[MAXMOTORS] = {0,1,2,3};
#endif
void CH_out_Mapping_Fun(u16 *out,u16 *mapped )
{
	u8 i;
	for( i = 0 ; i < MAXMOTORS ; i++ )
	{
		*( mapped + i ) = *( out + CH_out_Mapping[i] );
	}
}
u8 sel=0;
int16_t pwm_test=0;
void SetPwm(int16_t pwm[MAXMOTORS],s16 min,s16 max)
{
	u8 i;
	#if USE_MINI_BOARD
	s16 pwm_tem[12];
  #else
	s16 pwm_tem[MAXMOTORS];
	#endif
	for(i=0;i<MAXMOTORS;i++)
	{
		#if defined(DEBUG_MODE)
		  pwm_tem[i] = 0 ;
		#else
			pwm_tem[i] = pwm[i] ;
		#endif
			pwm_tem[i] = LIMIT(pwm_tem[i],min,max);
	}
	
#if USE_MINI_BOARD
	
	#if MAXMOTORS==4
	TIM3->CCR1 = PWM_RADIO *( pwm_tem[0] ) + INIT_DUTY;				//1	
	TIM3->CCR2 = PWM_RADIO *( pwm_tem[1] ) + INIT_DUTY;				//2
	TIM3->CCR3 = PWM_RADIO *( pwm_tem[2] ) + INIT_DUTY;				//3	
	TIM3->CCR4 = PWM_RADIO *( pwm_tem[3] ) + INIT_DUTY;				//4
	TIM4->CCR1 = PWM_RADIO *( pwm_tem[0] ) + INIT_DUTY;				//3	
	TIM4->CCR2 = PWM_RADIO *( pwm_tem[1] ) + INIT_DUTY;				//4
	TIM4->CCR3 = PWM_RADIO *( pwm_tem[2] ) + INIT_DUTY;				//3	
	TIM4->CCR4 = PWM_RADIO *( pwm_tem[3] ) + INIT_DUTY;				//4
	#else
	TIM3->CCR1 = PWM_RADIO *( pwm_tem[0] ) + INIT_DUTY;				//1	
	TIM3->CCR2 = PWM_RADIO *( pwm_tem[1] ) + INIT_DUTY;				//2
	TIM3->CCR3 = PWM_RADIO *( pwm_tem[2] ) + INIT_DUTY;				//3	
	TIM3->CCR4 = PWM_RADIO *( pwm_tem[3] ) + INIT_DUTY;				//4
	TIM4->CCR1 = PWM_RADIO *( pwm_tem[4] ) + INIT_DUTY;				//5
	TIM4->CCR2 = PWM_RADIO *( pwm_tem[5] ) + INIT_DUTY;				//6
	TIM4->CCR3 = PWM_RADIO *( pwm_tem[6] ) + INIT_DUTY;				//7	
	TIM4->CCR4 = PWM_RADIO *( pwm_tem[7] ) + INIT_DUTY;				//8
	 #if MAXMOTORS==12
	  TIM1->CCR1 = PWM_RADIO *( pwm_tem[8] ) + INIT_DUTY;				//9	
		TIM8->CCR1 = PWM_RADIO *( pwm_tem[9] ) + INIT_DUTY;				//10	
	  TIM8->CCR2 = PWM_RADIO *( pwm_tem[10] ) + INIT_DUTY;			//11	
		TIM8->CCR3 = PWM_RADIO *( pwm_tem[11] ) + INIT_DUTY;			//12
	 #endif
	#endif
#else
	TIM1->CCR4 = PWM_RADIO *( pwm_tem[0] ) + INIT_DUTY;				//1	
	TIM1->CCR2 = PWM_RADIO *( pwm_tem[2] ) + INIT_DUTY;				//2
	TIM1->CCR3 = PWM_RADIO *( pwm_tem[1] ) + INIT_DUTY;				//3	
	TIM1->CCR1 = PWM_RADIO *( pwm_tem[3] ) + INIT_DUTY;				//4
	TIM4->CCR1 = PWM_RADIO *( pwm_tem[4] ) + INIT_DUTY;				//3	
	TIM4->CCR2 = PWM_RADIO *( pwm_tem[5] ) + INIT_DUTY;				//4
#endif
}


void SetPwm_Fan(int16_t pwm[6])
{
	u8 i;
	s16 pwm_tem[6];
	for(i=0;i<4;i++)
	{
		  fan.out[i]=LIMIT(pwm[i]*fan.per_degree*fan.flag[i]+fan.off[i],fan.off[i]-fan.min[i],fan.off[i]+fan.max[i]);
	}
		#if defined(DEBUG_MODE)
		  pwm_tem[4] = 0 ;
	  #else
	    pwm_tem[4] = LIMIT(pwm[4],0,1000); 
		#endif		
	if(fan.test[0]!=0){	
	TIM3->CCR1 = fan.test[0];				//1	
	TIM3->CCR2 = fan.test[1];				//2
	TIM3->CCR3 = fan.test[2];				//3	
	TIM3->CCR4 = fan.test[3];				//4		
	}else{
	TIM3->CCR1 = fan.out[0];				//1	
	TIM3->CCR2 = fan.out[1];				//2
	TIM3->CCR3 = fan.out[2];				//3	
	TIM3->CCR4 = fan.out[3];				//4
	}
	TIM4->CCR1 = PWM_RADIO *( pwm_tem[4] ) + INIT_DUTY;				//3	
	TIM4->CCR2 = PWM_RADIO *( 0 ) + INIT_DUTY;				//4
	TIM4->CCR3 = PWM_RADIO *( 0 ) + INIT_DUTY;				//3	
	TIM4->CCR4 = PWM_RADIO *( 0 ) + INIT_DUTY;				//4

}
const u32 MOTORS[] = { 0, 1, 2, 3 ,4 ,5 ,6, 7, 7, 9, 10, 11};
u8 motor_test=0;
void motorsTest400(float rate)
{
	int i;
  s16 pwm_tem[12];
	//for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	for (i = 0; i < MAXMOTORS; i++)	
	{		
		pwm_tem[0]=pwm_tem[1]=pwm_tem[2]=pwm_tem[3]=pwm_tem[4]=pwm_tem[5]=0;
		pwm_tem[6]=pwm_tem[7]=pwm_tem[8]=pwm_tem[9]=pwm_tem[10]=pwm_tem[11]=0;
		pwm_tem[i]=rate*1000;
		SetPwm(pwm_tem,0,1000);
		Delay_ms(MOTORS_TEST_ON_TIME_MS);
		pwm_tem[0]=pwm_tem[1]=pwm_tem[2]=pwm_tem[3]=pwm_tem[4]=pwm_tem[5]=0;
		pwm_tem[6]=pwm_tem[7]=pwm_tem[8]=pwm_tem[9]=pwm_tem[10]=pwm_tem[11]=0;
		SetPwm(pwm_tem,0,1000);
		Delay_ms(MOTORS_TEST_DELAY_TIME_MS);
	}
  motor_test=0;
}