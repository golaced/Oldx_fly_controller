#include "dog.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//独立看门狗 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/6/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//初始化独立看门狗
//prer:分频数:0~7(只有低3位有效!)
//rlr:自动重装载值,0~0XFFF.
//分频因子=4*2^prer.但最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(u8 prer,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	
	IWDG_SetPrescaler(prer); //设置IWDG分频系数

	IWDG_SetReload(rlr);   //设置IWDG装载值

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //使能看门狗
}

//喂独立看门狗
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}
