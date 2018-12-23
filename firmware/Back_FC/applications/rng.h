#ifndef __RNG_H
#define __RNG_H	 
#include "stm32f4xx.h" 
 //////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//RNG(随机数发生器)驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/5
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

	
u8  RNG_Init(void);			//RNG初始化 
u32 RNG_Get_RandomNum(void);//得到随机数
int RNG_Get_RandomRange(int min,int max);//生成[min,max]范围的随机数
#endif

















