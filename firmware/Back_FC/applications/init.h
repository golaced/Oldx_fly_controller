#ifndef _INIT_H_
#define _INIT_H_

#include "stm32f4xx.h"

u8 All_Init(void);
extern u8 Init_Finish;


typedef struct 
{
  u8 baro_ekf;
	u8 imu_board;
	u8 mems;
}_SYS_INIT;

extern _SYS_INIT sys_init;
#endif
