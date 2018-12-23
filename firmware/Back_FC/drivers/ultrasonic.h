#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "stm32f4xx.h"
#include "ms5611.h"


void Ultrasonic_Init(void);
void Ultra_Duty(void);
void Ultra_Get(u8);

extern s8 ultra_start_f;

extern _height_st ultra;

#endif


