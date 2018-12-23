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

#ifndef _CKF_H
#define _CKF_H

#include "Matrix.h"

//7-state q0 q1 q2 q3 wx wy wz
#define CKF_STATE_DIM 7
//13-measurement q0 q1 q2 q3 ax ay az wx wy wz mx my mz
#define CKF_MEASUREMENT_DIM 13

#define CKF_CP_POINTS 14//(2 * CKF_STATE_DIM)

#define CKF_HALFPI 1.5707963267948966192313216916398f
#define CKF_PI 3.1415926535897932384626433832795f
#define CKF_TWOPI 6.283185307179586476925286766559f
#define CKF_TODEG(x) ((x) * 57.2957796f)

typedef struct CKF_FILTER_T{
	//weights
	float32_t W;
	//Kesi
	float32_t Kesi_f32[CKF_STATE_DIM * CKF_CP_POINTS];
	float32_t iKesi_f32[CKF_STATE_DIM];
	//state covariance
	float32_t P_f32[CKF_STATE_DIM * CKF_STATE_DIM];
	float32_t PX_f32[CKF_STATE_DIM * CKF_STATE_DIM];
	float32_t PY_f32[CKF_MEASUREMENT_DIM * CKF_MEASUREMENT_DIM];
	float32_t tmpPY_f32[CKF_MEASUREMENT_DIM * CKF_MEASUREMENT_DIM];
	float32_t PXY_f32[CKF_STATE_DIM * CKF_MEASUREMENT_DIM];
	float32_t tmpPXY_f32[CKF_STATE_DIM * CKF_MEASUREMENT_DIM];
	float32_t Q_f32[CKF_STATE_DIM * CKF_STATE_DIM];
	float32_t R_f32[CKF_MEASUREMENT_DIM * CKF_MEASUREMENT_DIM];
	//cubature points
	float32_t XCP_f32[CKF_STATE_DIM * CKF_CP_POINTS];
	float32_t XminusCP_f32[CKF_STATE_DIM * CKF_CP_POINTS];
	float32_t YSP_f32[CKF_MEASUREMENT_DIM * CKF_CP_POINTS];

	//Kalman gain
	float32_t K_f32[CKF_STATE_DIM * CKF_MEASUREMENT_DIM];
	float32_t KT_f32[CKF_MEASUREMENT_DIM * CKF_STATE_DIM];
	//state vector
	float32_t X_f32[CKF_STATE_DIM];
	float32_t XT_f32[CKF_STATE_DIM];
	float32_t Xminus_f32[CKF_STATE_DIM];
	float32_t XminusT_f32[CKF_STATE_DIM];
	float32_t tmpX_f32[CKF_STATE_DIM];
	float32_t tmpS_f32[CKF_STATE_DIM];
	//measurement vector
	float32_t Y_f32[CKF_MEASUREMENT_DIM];
	float32_t YT_f32[CKF_MEASUREMENT_DIM];
	float32_t Yminus_f32[CKF_MEASUREMENT_DIM];
	float32_t YminusT_f32[CKF_MEASUREMENT_DIM];
	float32_t tmpY_f32[CKF_MEASUREMENT_DIM];
	//
	arm_matrix_instance_f32 Kesi;
	arm_matrix_instance_f32 iKesi;
	arm_matrix_instance_f32 P;
	arm_matrix_instance_f32 PX;
	arm_matrix_instance_f32 PY;
	arm_matrix_instance_f32 tmpPY;
	arm_matrix_instance_f32 PXY;
	arm_matrix_instance_f32 tmpPXY;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 R;
	//
	arm_matrix_instance_f32 XCP;
	arm_matrix_instance_f32 XminusCP;
	arm_matrix_instance_f32 YCP;
	//
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 KT;
	//
	arm_matrix_instance_f32 X;
	arm_matrix_instance_f32 XT;
	arm_matrix_instance_f32 Xminus;
	arm_matrix_instance_f32 XminusT;
	arm_matrix_instance_f32 tmpX;
	arm_matrix_instance_f32 tmpS;
	arm_matrix_instance_f32 Y;
	arm_matrix_instance_f32 YT;
	arm_matrix_instance_f32 Yminus;
	arm_matrix_instance_f32 YminusT;
	arm_matrix_instance_f32 tmpY;
}CKF_Filter;

void CKF_New(CKF_Filter* ckf);
void CKF_Init(CKF_Filter* ckf, float32_t *q, float32_t *gyro);
void CKF_Update(CKF_Filter* ckf, float32_t *q, float32_t *gyro, float32_t *accel, float32_t *mag, float32_t dt);
void CKF_GetAngle(CKF_Filter* ckf, float32_t* rpy);

__inline void CKF_GetQ(CKF_Filter* ckf, float32_t* Q)
{
	Q[0] = ckf->X_f32[0];
	Q[1] = ckf->X_f32[1];
	Q[2] = ckf->X_f32[2];
	Q[3] = ckf->X_f32[3];
}

#endif
