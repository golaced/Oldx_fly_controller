#ifndef _RC_H
#define	_RC_H

#include "stm32f4xx.h"

void Fly_Ready(float T,float height_speed_mm);
void RC_Duty(float , u16 * );
void Feed_Rc_Dog(u8 ch_mode);
void Mode(void);

extern float CH_filter[];
extern s16 CH[];
extern u8 fly_ready,NS,thr_stick_low  ;
extern u8 height_ctrl_mode ;
extern u16 RX_CH[];
extern u8 rc_lose;
extern u8 NS;
extern u16 RX_CH_PWM[];
extern int RX_CH_FIX[4];
extern int RX_CH_FIX_PWM[4];
#endif

