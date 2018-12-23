#ifndef __HEIGHT_CTRL_H
#define __HEIGHT_CTRL_H

#include "stm32f4xx.h"

#include "parameter.h"

#define _xyz_f_t xyz_f_t

#define THR_TAKE_OFF_LIMIT 450
typedef struct
{
	float m_acc;
	float m_speed;
	float m_height;
	float fusion_acc;
	float fusion_speed;
	float fusion_height;

}_hc_value_st;
extern float ultra_ctrl_out_use,ultra_dis_lpf;
extern _hc_value_st hc_value;
extern u8 height_ctrl_mode;
float auto_take_off_land(float dT,u8 ready);
float Height_Ctrl(float T,float thr,u8 ready,float en);
void h_pid_init(void);
extern int exp_height;
extern float ALT_VEL_SONAR,ALT_POS_SONAR,ALT_POS_SONAR2,ALT_VEL_BMP,ALT_POS_BMP,ALT_VEL_BMP_EKF,ALT_POS_BMP_EKF;
extern int baroAlt,ultra_distance;
typedef struct
{ float exp;
	float err;
	float err_old;
	float err_d;
	float err_i;
	float pid_out;
	float err_weight;
	float now;
  u8 hold_alt_flag;
}_st_height_pid_v;

typedef struct
{
	float kp;
	float kd;
	float ki;
  float fp;
	float ero_r[3];
}_st_height_pid;

void Height_Ctrl1(float T,float thr);

void Ultra_PID_Init(void);

void WZ_Speed_PID_Init(void);

void height_speed_ctrl1(float T,float thr,float exp_z_speed,float );

void Baro_Ctrl1(float T,float thr);

void Ultra_Ctrl1(float T,float thr);
void Ultra_Ctrl2(float T,float thr);
extern float ultra_ctrl_out;

extern float height_ctrl_out;

extern u8 baro_ctrl_start;

extern float ultra_speed,wz_speed,baro_height;
#define ULTRA_SPEED 		 300    // mm/s
#define ULTRA_MAX_HEIGHT 1500   // mm
#define ULTRA_INT        300    // »ý·Ö·ù¶È

#define H_INNER 10
#define F_POS_SPD 20
#define F_POS 2
#define H_OUTTER 2*H_INNER
#define MAX_VERTICAL_SPEED 1500.0
extern _st_height_pid_v wz_speed_pid_v;
extern _st_height_pid wz_speed_pid;
extern _st_height_pid_v ultra_ctrl;
extern _st_height_pid ultra_pid;
extern _st_height_pid track_pid[10];
int height_test(float T);
#endif

