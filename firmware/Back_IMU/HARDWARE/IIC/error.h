#ifndef _ERROR_H_
#define	_ERROR_H_

#include "stm32f4xx.h"


typedef struct 
{
 u8 ero_att,ero_hight,ero_rst_h,ero_rst_att;
	
}ERO;

extern ERO ero;

typedef struct 
{
 u8 mpu6050;
 u8 hml5833;
 u8 ms5611;
 u8 flow;
 u8 circle;
 u8 gps;
 u8 sonar;
 u8 avoid;	
}SYS_STATE;

extern SYS_STATE sys;
void att_ero_check(void);
void hight_ero_check(void);
#endif

