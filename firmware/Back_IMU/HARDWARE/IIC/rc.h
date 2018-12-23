#ifndef _RC_H
#define	_RC_H

#include "stm32f4xx.h"
#include "include.h"
#define CH_NUM 				(8) 	//接收机通道数量
void Fly_Ready(float T);
void RC_Duty(float , u16 * );
void Feed_Rc_Dog(u8 ch_mode);
void Mode_FC(void);

extern float CH_filter[];
extern s16 CH[];
extern u8 fly_ready,fly_ready_rx,NS ;
extern u8 height_ctrl_mode,height_ctrl_mode_use ;
\
extern u16 RX_CH[CH_NUM];
extern int RX_CH_FIX[4];
#endif

