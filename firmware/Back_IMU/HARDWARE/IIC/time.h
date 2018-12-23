#ifndef _TIME_H_
#define _TIME_H_

#include "stm32f4xx.h"

void TIM_INIT(void);
void sys_time(void);

u16 Get_Time(u8,u16,u16);

float Get_Cycle_T(u8 );

void Cycle_Time_Init(void);

extern volatile uint32_t sysTickUptime;
extern int time_1h,time_1m,time_1s,time_1ms;

void Delay_us(uint32_t);
void Delay_ms(uint32_t);
void SysTick_Configuration(void);
uint32_t GetSysTime_us(void);

void TIM7_Int_Init(u16 arr,u16 psc);
void Initial_Timer_SYS(void);
uint32_t micros(void);
void TIM3_Int_Init(u16 arr,u16 psc);

#define GET_T_OUTTER 0
#define GET_T_INNER 1
#define GET_T_FLOW_UART 2
#define GET_T_SONAR_SAMPLE 3
#define GET_T_BARO_UKF 4
#define GET_T_EKF 5
#define GET_T_MEMS 6
#define GET_T_MEMS_INT 7
#define GET_T_FLOW 8
#define GET_T_FLOW_UKF 9
#define GET_T_HIGH_CONTROL_O 10
#define GET_T_BARO_EKF 11
#define GET_T_GPS 12
#define GET_T_FLOW_SAMPLE 13
#define GET_T_UART 14
#define GET_T_FC 15
#define GET_T_M100 16
#define GET_T_PVT 17
#define GET_T_UKF_GPS 18
#define GET_T_UKF_FLOW 19
#define TIME_UPDATE 20

#define TEST 29
#endif
