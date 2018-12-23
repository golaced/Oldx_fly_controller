#ifndef _MultiRotor_rc_H_
#define _MultiRotor_rc_H_
#include "stm32f4xx.h"

void Nrf_Check_Event(void);
void NRF_Send_AF(void);
void NRF_Send_AE(void);
void NRF_Send_OFFSET(void);
void NRF_Send_PID(void);
void NRF_Send_ARMED(void);
void NRF_SEND_test(void);
extern unsigned int cnt_timer2;
extern void RC_Send_Task(void);
extern u8 key_rc[6];
extern u16 data_rate;
extern void CAL_CHECK(void);


typedef struct 
{ 
  float spd[3];
	float pos[3];
	float acc_nez[3];
	float acc_body[3];
	float bat;
	u8 fly_mode;
	u8 acc_3d_step;
	u8 fall,knock;
}_DRONE;

extern _DRONE drone;
void param_copy_pid(void);
void pid_copy_param(void);
#endif
