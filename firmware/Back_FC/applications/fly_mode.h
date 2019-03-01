#ifndef __FLY_MODE_H
#define __FLY_MODE_H

#include "stm32f4xx.h"
#include "include.h"
#include "parameter.h"





enum
{
	BARO=0,
	GPS,
	BACK_HOME,
	//UTRASONIC,

};
typedef struct 
{
 u8 ero_att,ero_hight,ero_rst_h,ero_rst_att;
 u8 baro_ekf,baro_ekf_cnt;
	
}ERO;
extern ERO ero;
extern u8 mode_value[],mode_state;

typedef struct 
{
 u8 gps,flow,sonar,bmp,laser,pi,pi_flow,acc_imu,gyro_imu,hml_imu,nrf,flash,system;
}MOUDLE;
extern MOUDLE module;
void mode_check(float *ch_in,u8 *mode_value);
extern u8 height_ctrl_mode_rc;
#endif
