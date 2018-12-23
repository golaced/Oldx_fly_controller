#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "stm32f4xx.h"

void Ultrasonic_Init(void);
void Ultra_Duty(void);
void Ultra_Duty_SCL(void);
void Ultra_Get(u8);

extern s8 ultra_start_f;
extern int ultra_distance,ultra_distance_r;
extern float ultra_delta;
extern double x_pred ; // m   0
extern double v_pred ; //       1
extern u8 ultra_ok ;
extern float sonar_filter(float hight,float dt_sonar);

extern double x_pred_bmp ; // m   0
extern double v_pred_bmp  ; //       1
extern float sonar_filter_bmp (float hight,float dt_sonar);
#endif


