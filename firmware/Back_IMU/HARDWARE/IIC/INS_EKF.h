/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _INS_EKF_H_
#define _INS_EKF_H_

#include "Matrix.h"

//16-state q0 q1 q2 q3 Pn Pe Alt Vn Ve Vd bwx bwy bwz bax bay baz
#define INS_EKF_STATE_DIM 16

//9-measurement mx my mz (3D magnetometer) Pn Pe Alt Vn Ve Vd
//unit vector pointing to MagNorth in body coords
//north pos, east pos, altitude
//north vel, east vel, down velocity
#define INS_EKF_MEASUREMENT_DIM 9

#define INS_EKF_HALFPI 1.5707963267948966192313216916398f
#define INS_EKF_PI 3.1415926535897932384626433832795f
#define INS_EKF_TWOPI 6.283185307179586476925286766559f
#define INS_EKF_TODEG(x) ((x) * 57.2957796f)

typedef struct INS_EKF_FILTER_T{
	//state covariance
	float32_t P_f32[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	float32_t PX_f32[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	float32_t PHT_f32[INS_EKF_STATE_DIM * INS_EKF_MEASUREMENT_DIM];
	
	float32_t Q_f32[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	float32_t R_f32[INS_EKF_MEASUREMENT_DIM * INS_EKF_MEASUREMENT_DIM];
	
	float32_t K_f32[INS_EKF_STATE_DIM * INS_EKF_MEASUREMENT_DIM];
	float32_t KH_f32[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	float32_t KHP_f32[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	
	float32_t KY_f32[INS_EKF_STATE_DIM];

	float32_t F_f32[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	float32_t FT_f32[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	//The H matrix maps the measurement to the states
	float32_t H_f32[INS_EKF_MEASUREMENT_DIM * INS_EKF_STATE_DIM];
	float32_t HT_f32[INS_EKF_STATE_DIM * INS_EKF_MEASUREMENT_DIM];
	float32_t HP_f32[INS_EKF_MEASUREMENT_DIM * INS_EKF_STATE_DIM];
	
	float32_t S_f32[INS_EKF_MEASUREMENT_DIM * INS_EKF_MEASUREMENT_DIM];
	float32_t SI_f32[INS_EKF_MEASUREMENT_DIM * INS_EKF_MEASUREMENT_DIM];
	//state vector
	float32_t X_f32[INS_EKF_STATE_DIM];
	//measurement vector
	float32_t Y_f32[INS_EKF_MEASUREMENT_DIM];

	arm_matrix_instance_f32 P;
	arm_matrix_instance_f32 PX;
	arm_matrix_instance_f32 PHT;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 R;
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 KH;
	arm_matrix_instance_f32 KHP;
	arm_matrix_instance_f32 KY;
	arm_matrix_instance_f32 F;
	arm_matrix_instance_f32 FT;
	arm_matrix_instance_f32 H;
	arm_matrix_instance_f32 HT;
	arm_matrix_instance_f32 HP;
	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 SI;
	
	arm_matrix_instance_f32 X;
	arm_matrix_instance_f32 Y;
	
	float32_t declination;
	float32_t gravity; //m/s^2
}INS_EKF_Filter;

void INS_EKF_New(INS_EKF_Filter* ins);
void INS_EKF_Init(INS_EKF_Filter* ins, float32_t *p, float32_t *v, float32_t *accel, float32_t *mag);
void INS_EFK_Update(INS_EKF_Filter* ins, float32_t *mag, float32_t *p, float32_t *v, float32_t *gyro, float32_t *accel, float32_t dt);
void INS_EKF_GetAngle(INS_EKF_Filter* ins, float32_t* rpy);

__inline void INS_EKF_GetPos(INS_EKF_Filter* ins, float32_t* x)
{
	x[0] = ins->X_f32[4];
	x[1] = ins->X_f32[5];
	x[2] = ins->X_f32[6];
}

#endif
