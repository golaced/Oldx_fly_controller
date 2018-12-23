
#include "time.h"
#include "include.h"

volatile uint32_t sysTickUptime = 0;

#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)

void  SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t         cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;

	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

uint32_t GetSysTime_us(void) 
{
//	register uint32_t ms;
//	u32 value;
//	ms = sysTickUptime;
//	value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return micros();
}

void Delay_us(uint32_t us)
{
 delay_us(1);
}

void Delay_ms(uint32_t ms)
{
    while (ms--)
        Delay_us(1000);
}

int time_1h,time_1m,time_1s,time_1ms;


volatile float Cycle_T[GET_TIME_NUM][3];

enum
{
	NOW = 0,
	OLD,
	NEW,
};

float Get_Cycle_T(u8 item)	
{
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//上一次的时间
	Cycle_T[item][NOW] = (float)micros()/1000000.0f; //GetSysTime_us()/1000000.0f; //本次的时间
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//间隔的时间（周期）
	return Cycle_T[item][NEW];
}

void Cycle_Time_Init()
{
	u8 i;
	for(i=0;i<GET_TIME_NUM;i++)
	{
		Get_Cycle_T(i);
	}

}



/**************************实现函数********************************************
*函数原型:		void Initial_Timer3(void)
*功　　能:	  初始化Tim2  Tim3 将两个定时器级联，以产生一个32位的定时器来提供系统us 级的计时	
输入参数：无
输出参数：没有	
*******************************************************************************/
void Initial_Timer_SYS(void)
{
  RCC->APB1ENR |= 0x0008;	//使能TIM5时钟
	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
	TIM5->CR2 = 0x0000;
	TIM5->CNT = 0x0000;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->PSC = 84 - 1;	//分出 1M 的时钟 保证每个周期为1us
	TIM5->EGR = 0x0001;
	TIM5->CR1 |= 0x0001; //启动定时器           
}


/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM5->CNT;
 	return temp;
}



void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}



//定时器7中断服务程序		    
void TIM7_IRQHandler(void)
{ 	
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//是更新中断
	{	 			   
		USART3_RX_STA|=1<<15;	//标记接收完成
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //清除TIM7更新中断标志    
		TIM_Cmd(TIM7, DISABLE);  //关闭TIM7 
	}	    
}
 
//通用定时器7中断初始化
//这里时钟选择为APB1的2倍，而APB1为42M
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz 
//通用定时器中断初始化
//这里始终选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数		 
void TIM7_Int_Init(u16 arr,u16 psc)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//TIM7时钟使能    
	
	//定时器TIM7初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE ); //使能指定的TIM7中断,允许更新中断
	
	TIM_Cmd(TIM7,ENABLE);//开启定时器7
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
}
	 

