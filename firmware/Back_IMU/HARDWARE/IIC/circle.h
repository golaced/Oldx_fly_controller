#ifndef _CIRCLE_H_
#define	_CIRCLE_H_

#include "stm32f4xx.h"



 typedef struct
{
 int x,y;
 int x_flp,y_flp;
 u8 check;
 u8 connect,lose_cnt;

}CIRCLE;
extern CIRCLE circle;
extern float nav_circle[2];
extern void circle_control(float T);
#define MID_Y 152-40+14
#define MID_X 182+60-38
#endif