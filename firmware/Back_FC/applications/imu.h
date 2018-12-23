#ifndef _IMU_H_
#define	_IMU_H_

#include "stm32f4xx.h"

#include "parameter.h"
#include "mymath.h"
#include "math.h"

typedef struct 
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;
	
}ref_t;
extern float Pit_fc1,Rol_fc1;
extern xyz_f_t reference_v,acc_3d_hg;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw);
extern float  Roll,Pitch,Yaw,Roll_mid_down,Pitch_mid_down,Yaw_mid_down,Rol_fc,Pit_fc,Yaw_fc;   
extern float  ref_q[4],q_nav[4],ref_q_imd_down[4];
extern float reference_vr_imd_down[3],reference_vr[3];
void MadgwickAHRSupdate(float dt,float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float *rol,float *pit,float *yaw);
void MadgwickAHRSupdateIMU(float dt,float gx, float gy, float gz, float ax, float ay, float az);
extern float Yaw_fc_q;
extern float ref_q_imd_down_fc[4] ;
extern float reference_vr_imd_down_fc[3];

#endif

