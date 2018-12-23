#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "stm32f4xx.h"
#define MAXMOTORS 		(4)		//电机数量
u8 PWM_Out_Init(uint16_t hz);
void SetPwm(int16_t pwm[],s16 min,s16 max);
void  Set_DJ(float ang1,float ang2,float ang3,float ang4);
#endif

