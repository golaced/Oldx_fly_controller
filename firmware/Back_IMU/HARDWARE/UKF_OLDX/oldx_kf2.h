
#include "stm32f4xx.h"
#include <math.h>

u8 OLDX_KF2(float *measure,float tau,float *r_sensor,u8 *flag_sensor,double *state,double *state_correct,float T);