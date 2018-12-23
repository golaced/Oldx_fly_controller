#include "rng.h"
#include "stm32f4xx_rng.h"
#include "time.h"

//初始化RNG
u8 RNG_Init(void)
{
	u16 retry=0; 
	
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);//开启RNG时钟,来自PLL48CLK
	
	RNG_Cmd(ENABLE);	//使能RNG
	
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET&&retry<10000)	//等待随机数就绪
	{
		retry++;
		Delay_us(100);
	}
	if(retry>=10000)return 1;//随机数产生器工作不正常
	return 0;
}
//得到随机数
//返回值:获取到的随机数
u32 RNG_Get_RandomNum(void)
{	 
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET);	//等待随机数就绪  
	return RNG_GetRandomNumber();	
}

//生成[min,max]范围的随机数
int RNG_Get_RandomRange(int min,int max)
{ 
   return RNG_Get_RandomNum()%(max-min+1) +min;
}



