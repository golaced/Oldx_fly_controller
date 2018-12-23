#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "FastMath.h"

void Quaternion_Normalize(float *q);
void Quaternion_FromEuler(float *q, float *rpy);
void Quaternion_ToEuler(float *q, float* rpy);
void Quaternion_FromRotationMatrix(float *R, float *Q);
void Quaternion_RungeKutta4(float *q, float *w, float dt, int normalize);
void Quaternion_From6AxisData(float* q, float *accel, float *mag);
void Quaternion_To_R(float *q, float R[3][3]);
void Eular_FromRotationMatrix(float R[3][3], float *rpy);
void Quaternion_ToNumQ( float *q, float *rpy);
void Quaternion_To_R2(float *q, float R[3][3]);

void q_to_dcm(float data[4],float dcm[3][3]);
void dcm_to_q(float q[4] ,float dcm[3][3]);
void euler_to_q(float angle[3],float q[4]) ;
void  q_to_euler(float data[4],float angle[3]);
void dcm_to_euler(float euler[3],float data[3][3] );
void cal_ero_outter_so3(void);
extern float ero_angle_px4[4];

extern float R_control_now[3][3];
#define PX4_ARRAY2D(_array, _ncols, _x, _y) (_array[_x * _ncols + _y])
#define PX4_R(_array, _x, _y) PX4_ARRAY2D(_array, 3, _x, _y)




typedef struct
{
 float pt[3];
 float vt[3];
 float at[3];
 float ps[3];
 float vs[3];
 float as[3];
 float pe[3];
 float ve[3];
 float ae[3];
 float param[10];
 float Time,time_now;
 float cost,cost_all;
 char defined[3];
	
}_TRA;

extern _TRA traj[10];

void plan_tra(_TRA *tra);
void get_tra(_TRA *tra,float t);

#endif
