#ifndef __ANOTC_BARO_H
#define __ANOTC_BARO_H

#include "stm32f4xx.h"
#include "filter.h"
#include "imu.h"
#include "mymath.h"

#include "parameter.h"

#include "height_ctrl.h"

#include "ultrasonic.h"

//typedef xyz_f_t _xyz_f_t;
#define _xyz_f_t xyz_f_t

/*=====================================================================================================================
						 *****
=====================================================================================================================*/
typedef struct
{
	float b1;
	float b2;
	float b3;
	
	float g1;
	float g2;
	float g3;

}_f_set_st;

typedef struct
{
	float est_acc_old;
	_filter_1_st fusion_acceleration;
	_filter_1_st fusion_speed_m;
	_filter_1_st fusion_speed_me;
	_filter_1_st fusion_displacement;

}_fusion_st;
extern _fusion_st sonar_fusion;
extern _fusion_st baro_fusion;

typedef struct
{
	float dis_deadzone;
	float displacement;
	float displacement_old;
	float speed;
	float speed_old;
	float acceleration;
	
}_fusion_p_st;
extern _fusion_p_st baro_p;

typedef struct
{
	u8 item;
	float a;
	float b;
	float c;
}_h_f_set_st;
				

extern float wz_speed;
extern _height_st sonarf;	
float baro_compensate(float dT,float kup,float kdw,float vz,float lim);
void baro_ctrl(float dT,_hc_value_st *);

#endif

