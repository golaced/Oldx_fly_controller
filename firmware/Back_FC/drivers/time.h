#ifndef _TIME_H_
#define _TIME_H_

#include "stm32f4xx.h"

void TIM_INIT(void);
void sys_time(void);

u16 Get_Time(u8,u16,u16);

u32 Get_Cycle_T(u8 );

void Cycle_Time_Init(void);

extern volatile uint32_t sysTickUptime;
extern int time_1h,time_1m,time_1s,time_1ms;

void Delay_us(uint32_t);
void Delay_ms(uint32_t);
void SysTick_Configuration(void);
uint32_t GetSysTime_us(void);
void TIM3_Int_Init(u16 arr,u16 psc);
#define GET_T_OUTTER 0
#define GET_T_INNER 1
#define GET_T_HIGH_CONTROL_I 2
#define GET_T_HIGH_CONTROL_O 3
#define GET_T_BARO 4
#define GET_T_OUT_NAV 5
#define GET_T_IN_NAV 6
#define GET_T_HIGH_CONTROL 7
#endif
