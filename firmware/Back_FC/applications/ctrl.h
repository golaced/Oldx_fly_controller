#ifndef _CTRL_H
#define	_CTRL_H

#include "stm32f4xx.h"
#include "eso.h"
#include "pwm_out.h"
#include "rc.h"
#include "imu.h"
#include "mpu6050.h"

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
		PID4,
		PID5,
		PID6,

		PIDITEMS
};



typedef struct
{
	xyz_f_t err;
	xyz_f_t err_old;
	xyz_f_t err_i;
	xyz_f_t eliminate_I;
	xyz_f_t err_d;
	xyz_f_t damp;
	xyz_f_t out;
	pid_t 	PID[PIDITEMS];
	xyz_f_t err_weight;
	float FB;
  float zrate_limit;
}ctrl_t;

typedef struct
{
float min[4];
float	max[4];
float	off[4];
float out[4],test[4];
int flag[4];
float	per_degree;
float thr;

}FAN;

extern FAN fan;
extern ctrl_t ctrl_1;
extern ctrl_t ctrl_2;
extern xyz_f_t except_AS;
void CTRL_2(float);
void CTRL_1(float);
void Ctrl_Para_Init(void);
void Thr_Ctrl(float);
void All_Out(float x,float y,float z,float T);

extern u8 Thr_Low,force_Thr_low;
extern float Thr_Weight,thr_view;
extern s16 motor_out[MAXMOTORS];
extern float thr_value,k_pitch;
extern xyz_f_t except_A ;

#define YAW_ERO_MAX 45
#endif

