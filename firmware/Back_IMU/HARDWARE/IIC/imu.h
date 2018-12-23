#ifndef _IMU_H_
#define	_IMU_H_

#include "stm32f4xx.h"

#include "parameter.h"
#include "my_math.h"
#include "math.h"
#include "MadgwickAHRS.h"
typedef struct 
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;
	
}ref_t;
extern u8 init_ahrs;
extern float yaw_qr_off;
extern xyz_f_t mag_sim_3d,acc_3d_hg,acc_ng,acc_ng_offset;
extern float Roll_mid_down,Pitch_mid_down,Yaw_mid_down;    				//вкл╛╫г
extern xyz_f_t reference_v;
extern float yaw_mag_view[5];
extern double X_kf_yaw[2];
extern float reference_vr[3];
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw);
extern float Roll,Pitch,Yaw;
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) ;
extern float ref_q[4] , q_nav[4];


void MargAHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float accelCutoff, uint8_t magDataUpdate, float dt,float *rol,float *pit,float *yaw);
										
int madgwick_update_new(float ax,float ay,float az, float wx,float wy,float wz, float mx,float my ,float mz,float *rol,float *pit,float *yaw,float T);									
extern u8 init_hml_norm;
extern u16 init_hml_norm_cnt;


extern float q_so3[4];										
void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt,float *rol,float *pit,float *yaw);
										
#endif

