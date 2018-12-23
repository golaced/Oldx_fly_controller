#ifndef _CTRL_H
#define	_CTRL_H

#include "stm32f4xx.h"
#include "include.h"
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

}ctrl_t;

extern ctrl_t ctrl_1;
extern ctrl_t ctrl_2;
extern ctrl_t ctrl_2_fuzzy;
extern xyz_f_t except_AS;
void CTRL_2(float);
void CTRL_2_FUZZY(float);

void CTRL_1(float);
void Ctrl_Para_Init(void);
void Thr_Ctrl(float);
void All_Out(float x,float y,float z);

extern u8 Thr_Low;
extern float Thr_Weight;
extern float motor[MAXMOTORS];
extern xyz_f_t ctrl_angle_offset;
extern float thr_test;
extern void GPS_calc_poshold(float T);

extern xyz_f_t except_A ;

extern float actual_speed[2];
extern float tar_speed[2];
extern  float  now_position[2], target_position[2];
extern float nav[2];
extern float tar_speed[2];
extern float thr_value ,integrator[2];;
#define LON 0
#define LAT 1
extern float nav_angle_ukf[2],nav_ukf_g[2];
extern int baro_to_ground;
extern u16 att_tuning_thr_limit;
extern float w_neuro[2];
#endif

