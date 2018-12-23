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

#ifndef _UKF_H
#define _UKF_H

#include "Matrix.h"

//7-state q0 q1 q2 q3 wx wy wz
#define UKF_STATE_DIM 7
//13-measurement q0 q1 q2 q3 ax ay az wx wy wz mx my mz
#define UKF_MEASUREMENT_DIM 13

#define UKF_SP_POINTS 15//(2 * UKF_STATE_DIM + 1)

#define UKF_HALFPI 1.5707963267948966192313216916398f
#define UKF_PI 3.1415926535897932384626433832795f
#define UKF_TWOPI 6.283185307179586476925286766559f
#define UKF_TODEG(x) ((x) * 57.2957796f)

typedef struct UKF_FILTER_T{
	//scaling factor
	float32_t gamma;
	//weights for means
	float32_t Wm0, Wmi;
	//weights for covariance
	float32_t Wc_f32[UKF_SP_POINTS * UKF_SP_POINTS];
	//state covariance
	float32_t P_f32[UKF_STATE_DIM * UKF_STATE_DIM];
	float32_t PX_f32[UKF_STATE_DIM * UKF_STATE_DIM];
	float32_t PY_f32[UKF_MEASUREMENT_DIM * UKF_MEASUREMENT_DIM];
	float32_t tmpPY_f32[UKF_MEASUREMENT_DIM * UKF_MEASUREMENT_DIM];
	float32_t PXY_f32[UKF_STATE_DIM * UKF_MEASUREMENT_DIM];
	float32_t PXYT_f32[UKF_MEASUREMENT_DIM * UKF_STATE_DIM];
	float32_t Q_f32[UKF_STATE_DIM * UKF_STATE_DIM];
	float32_t R_f32[UKF_MEASUREMENT_DIM * UKF_MEASUREMENT_DIM];
	//Sigma points
	float32_t XSP_f32[UKF_STATE_DIM * UKF_SP_POINTS];
	float32_t tmpXSP_f32[UKF_STATE_DIM * UKF_SP_POINTS];
	float32_t tmpXSPT_f32[UKF_SP_POINTS * UKF_STATE_DIM];
	float32_t tmpWcXSP_f32[UKF_STATE_DIM * UKF_SP_POINTS];
	float32_t tmpWcYSP_f32[UKF_MEASUREMENT_DIM * UKF_SP_POINTS];
	float32_t YSP_f32[UKF_MEASUREMENT_DIM * UKF_SP_POINTS];
	float32_t tmpYSP_f32[UKF_MEASUREMENT_DIM * UKF_SP_POINTS];
	float32_t tmpYSPT_f32[UKF_SP_POINTS * UKF_MEASUREMENT_DIM];
	//Kalman gain
	float32_t K_f32[UKF_STATE_DIM * UKF_MEASUREMENT_DIM];
	float32_t KT_f32[UKF_MEASUREMENT_DIM * UKF_STATE_DIM];
	//state vector
	float32_t X_f32[UKF_STATE_DIM];
	float32_t tmpX_f32[UKF_STATE_DIM];
	//measurement vector
	float32_t Y_f32[UKF_MEASUREMENT_DIM];
	float32_t tmpY_f32[UKF_MEASUREMENT_DIM];
	//
	arm_matrix_instance_f32 Wc;
	arm_matrix_instance_f32 P;
	arm_matrix_instance_f32 PX;
	arm_matrix_instance_f32 PY;
	arm_matrix_instance_f32 tmpPY;
	arm_matrix_instance_f32 PXY;
	arm_matrix_instance_f32 PXYT;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 R;
	//
	arm_matrix_instance_f32 XSP;
	arm_matrix_instance_f32 tmpXSP;
	arm_matrix_instance_f32 tmpXSPT;
	arm_matrix_instance_f32 tmpWcXSP;
	arm_matrix_instance_f32 tmpWcYSP;
	arm_matrix_instance_f32 YSP;
	arm_matrix_instance_f32 tmpYSP;
	arm_matrix_instance_f32 tmpYSPT;
	//
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 KT;
	//
	arm_matrix_instance_f32 X;
	arm_matrix_instance_f32 tmpX;
	arm_matrix_instance_f32 Y;
	arm_matrix_instance_f32 tmpY;
}UKF_Filter;

void UKF_New(UKF_Filter* UKF);
void UKF_Init(UKF_Filter* UKF, float32_t *q, float32_t *gyro);
void UKF_Update(UKF_Filter* UKF, float32_t *q, float32_t *gyro, float32_t *accel, float32_t *mag, float32_t dt);
void UKF_GetAngle(UKF_Filter* UKF, float32_t* rpy);

__inline void UKF_GetQ(UKF_Filter* ukf, float32_t* Q)
{
	Q[0] = ukf->X_f32[0];
	Q[1] = ukf->X_f32[1];
	Q[2] = ukf->X_f32[2];
	Q[3] = ukf->X_f32[3];
}

#endif
