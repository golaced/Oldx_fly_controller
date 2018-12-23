#ifndef _PWM_IN_H_
#define _PWM_IN_H_

#include "stm32f4xx.h"

void PWM_IN_Init(void);
void _TIM3_IRQHandler(void);
void _TIM4_IRQHandler(void);

extern u16 Rc_Pwm_In[8];
#endif
