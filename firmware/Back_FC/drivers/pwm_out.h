#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "stm32f4xx.h"

u8 PWM_Out_Init(uint16_t hz);
u8 PWM_AUX_Out_Init(uint16_t hz);//50Hz
void SetPwm(int16_t pwm[],s16 min,s16 max);
void SetPwm_Fan(int16_t pwm[6]);
void SetPwm_AUX(float pit,float rol);

typedef struct 
{ u16 pwm_tem[4];
	int flag[4];
	u16 init[4];
	u16 max[4];
	u16 min[4];
	float att[4],att_ctrl[4],att_off[4];
	float pwm_per_dig[4];
	float ero[4],ero_reg[4];
}AUX_S;
void aux34(int c3,int c4);
extern AUX_S aux;

u8 cal_sita_form_pos(float r1,float r2,float x,float y,float z,float *sita1,float *sita2);
void motorsTest400(float rate);
extern u8 motor_test;
#define MOTORS_TEST_ON_TIME_MS    550
#define MOTORS_TEST_DELAY_TIME_MS 150
#endif

