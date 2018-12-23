#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "stm32f4xx.h"
void Outer_Compass_init(void);
void QMC5883_Update(void);
extern int mag_outer[3];
#endif

//------------------End of File----------------------------
