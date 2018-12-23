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

#ifndef _EKF_H
#define _EKF_H

#include "Matrix.h"

//7-state q0 q1 q2 q3 wx wy wz
#define EKF_STATE_DIM 7
//13-measurement q0 q1 q2 q3 ax ay az wx wy wz mx my mz
#define EKF_MEASUREMENT_DIM 13

#define EKF_HALFPI 1.5707963267948966192313216916398f
#define EKF_PI 3.1415926535897932384626433832795f
#define EKF_TWOPI 6.283185307179586476925286766559f
#define EKF_TODEG(x) ((x) * 57.2957796f)

typedef struct EKF_FILTER_T{
	//state covariance
	float32_t P_f32[EKF_STATE_DIM * EKF_STATE_DIM];
	float32_t Q_f32[EKF_STATE_DIM * EKF_STATE_DIM];
	float32_t R_f32[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
	//Kalman gain
	float32_t K_f32[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
	float32_t KT_f32[EKF_MEASUREMENT_DIM * EKF_STATE_DIM];
	//Measurement covariance
	float32_t S_f32[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
	//The H matrix maps the measurement to the states
	float32_t F_f32[EKF_STATE_DIM * EKF_STATE_DIM];
	float32_t FT_f32[EKF_STATE_DIM * EKF_STATE_DIM];
	float32_t H_f32[EKF_MEASUREMENT_DIM * EKF_STATE_DIM];
	float32_t HT_f32[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
	float32_t I_f32[EKF_STATE_DIM * EKF_STATE_DIM];
	//state vector
	float32_t X_f32[EKF_STATE_DIM];
	//measurement vector
	float32_t Y_f32[EKF_MEASUREMENT_DIM];
	//
	float32_t tmpP_f32[EKF_STATE_DIM * EKF_STATE_DIM];
	float32_t tmpS_f32[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
	float32_t tmpX_f32[EKF_STATE_DIM];
	float32_t tmpXX_f32[EKF_STATE_DIM * EKF_STATE_DIM];
	float32_t tmpXXT_f32[EKF_STATE_DIM * EKF_STATE_DIM];
	float32_t tmpXY_f32[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
	float32_t tmpYX_f32[EKF_MEASUREMENT_DIM * EKF_STATE_DIM];
	
	arm_matrix_instance_f32 P;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 R;
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 KT;
	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 F;
	arm_matrix_instance_f32 FT;
	arm_matrix_instance_f32 H;
	arm_matrix_instance_f32 HT;
	arm_matrix_instance_f32 I;
	//
	arm_matrix_instance_f32 X;
	arm_matrix_instance_f32 Y;
	//
	arm_matrix_instance_f32 tmpP;
	arm_matrix_instance_f32 tmpX;
	arm_matrix_instance_f32 tmpYX;
	arm_matrix_instance_f32 tmpXY;
	arm_matrix_instance_f32 tmpXX;
	arm_matrix_instance_f32 tmpXXT;
	arm_matrix_instance_f32 tmpS;
}EKF_Filter;

void EKF_New(EKF_Filter* ekf);
void EKF_Init(EKF_Filter* ekf, float32_t *q, float32_t *gyro);
void EFK_Update(EKF_Filter* ekf, float32_t *q, float32_t *gyro, float32_t *accel, float32_t *mag, float32_t dt);
void EKF_GetAngle(EKF_Filter* ekf, float32_t* rpy);

__inline void EKF_GetQ(EKF_Filter* efk, float32_t* Q)
{
	Q[0] = efk->X_f32[0];
	Q[1] = efk->X_f32[1];
	Q[2] = efk->X_f32[2];
	Q[3] = efk->X_f32[3];
}

#endif
