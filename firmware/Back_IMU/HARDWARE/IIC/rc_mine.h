#ifndef _MultiRotor_rc_H_
#define _MultiRotor_rc_H_
#include "include.h"

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
#endif
