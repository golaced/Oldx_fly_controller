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

#ifndef _SRCKF_H
#define _SRCKF_H

#include "Matrix.h"

//7-state q0 q1 q2 q3 wbx wby wbz
#define SRCKF_STATE_DIM 7
//6-measurement ax ay az mx my mz
#define SRCKF_MEASUREMENT_DIM 6

#define SRCKF_CP_POINTS 14//(2 * SRCKF_STATE_DIM)

#define SRCKF_HALFPI 1.5707963267948966192313216916398f
#define SRCKF_PI 3.1415926535897932384626433832795f
#define SRCKF_TWOPI 6.283185307179586476925286766559f
#define SRCKF_TODEG(x) ((x) * 57.2957796f)



typedef struct SRCKF_FILTER_T{
	//weights
	float32_t W;
	float32_t SW;
	//Kesi
	float32_t Kesi_f32[SRCKF_STATE_DIM * SRCKF_CP_POINTS];
	float32_t iKesi_f32[SRCKF_STATE_DIM];
	//state covariance
	float32_t S_f32[SRCKF_STATE_DIM * SRCKF_STATE_DIM];
	float32_t ST_f32[SRCKF_STATE_DIM * SRCKF_STATE_DIM];
	//measurement covariance
	float32_t SY_f32[SRCKF_MEASUREMENT_DIM * SRCKF_MEASUREMENT_DIM];
	float32_t SYI_f32[SRCKF_MEASUREMENT_DIM * SRCKF_MEASUREMENT_DIM];
	float32_t SYT_f32[SRCKF_MEASUREMENT_DIM * SRCKF_MEASUREMENT_DIM];
	float32_t SYTI_f32[SRCKF_MEASUREMENT_DIM * SRCKF_MEASUREMENT_DIM];
	//cross covariance
	float32_t PXY_f32[SRCKF_STATE_DIM * SRCKF_MEASUREMENT_DIM];
	float32_t tmpPXY_f32[SRCKF_STATE_DIM * SRCKF_MEASUREMENT_DIM];
	//
	float32_t SQ_f32[SRCKF_STATE_DIM * SRCKF_STATE_DIM];
	float32_t SR_f32[SRCKF_MEASUREMENT_DIM * SRCKF_MEASUREMENT_DIM];
	//cubature points
	float32_t XCP_f32[SRCKF_STATE_DIM * SRCKF_CP_POINTS];
	//propagated cubature points
	float32_t XPCP_f32[SRCKF_STATE_DIM * SRCKF_CP_POINTS];
	float32_t YPCP_f32[SRCKF_MEASUREMENT_DIM * SRCKF_CP_POINTS];
	//centered matrix
	float32_t XCPCM_f32[SRCKF_STATE_DIM * SRCKF_CP_POINTS];
	float32_t tmpXCPCM_f32[SRCKF_STATE_DIM * SRCKF_CP_POINTS];
	float32_t YCPCM_f32[SRCKF_MEASUREMENT_DIM * SRCKF_CP_POINTS];
	float32_t YCPCMT_f32[SRCKF_CP_POINTS * SRCKF_MEASUREMENT_DIM];
	float32_t XCM_f32[SRCKF_STATE_DIM * (SRCKF_CP_POINTS + SRCKF_STATE_DIM)];
	float32_t YCM_f32[SRCKF_MEASUREMENT_DIM * (SRCKF_CP_POINTS + SRCKF_MEASUREMENT_DIM)];
	float32_t XYCM_f32[SRCKF_STATE_DIM * (SRCKF_CP_POINTS + SRCKF_MEASUREMENT_DIM)];

	//Kalman gain
	float32_t K_f32[SRCKF_STATE_DIM * SRCKF_MEASUREMENT_DIM];
	//state vector
	float32_t X_f32[SRCKF_STATE_DIM];
	float32_t tmpX_f32[SRCKF_STATE_DIM];
	//measurement vector
	float32_t Y_f32[SRCKF_MEASUREMENT_DIM];
	float32_t tmpY_f32[SRCKF_MEASUREMENT_DIM];
	//
	//stuff
	//Kesi
	arm_matrix_instance_f32 Kesi;
	arm_matrix_instance_f32 iKesi;
	//state covariance
	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 ST;
	//measurement covariance
	arm_matrix_instance_f32 SY;
	arm_matrix_instance_f32 SYI;
	arm_matrix_instance_f32 SYT;
	arm_matrix_instance_f32 SYTI;
	//cross covariance
	arm_matrix_instance_f32 PXY;
	arm_matrix_instance_f32 tmpPXY;
	//
	arm_matrix_instance_f32 SQ;
	arm_matrix_instance_f32 SR;
	//cubature points
	arm_matrix_instance_f32 XCP;
	//propagated cubature points
	arm_matrix_instance_f32 XPCP;
	arm_matrix_instance_f32 YPCP;
	//centered matrix
	arm_matrix_instance_f32 XCPCM;
	arm_matrix_instance_f32 tmpXCPCM;
	arm_matrix_instance_f32 YCPCM;
	arm_matrix_instance_f32 YCPCMT;
	arm_matrix_instance_f32 XCM;
	arm_matrix_instance_f32 YCM;
	arm_matrix_instance_f32 XYCM;
	//Kalman gain
	arm_matrix_instance_f32 K;
	//state vector
	arm_matrix_instance_f32 X;
	arm_matrix_instance_f32 tmpX;
	//measurement vector
	arm_matrix_instance_f32 Y;
	arm_matrix_instance_f32 tmpY;
	
	float32_t declination;
}SRCKF_Filter;

void SRCKF_New(SRCKF_Filter* srckf);
void SRCKF_Init(SRCKF_Filter* srckf, float32_t *accel, float32_t *mag);
void SRCKF_Update(SRCKF_Filter* srckf, float32_t *gyro, float32_t *accel, float32_t *mag, float32_t dt);
void SRCKF_GetAngle(SRCKF_Filter* srckf, float32_t* rpy);
void SRCKF_Set_R(SRCKF_Filter* srckf,float ra,float rm);
__inline void SRCKF_GetQ(SRCKF_Filter* srckf, float32_t* Q)
{
	Q[0] = srckf->X_f32[0];
	Q[1] = srckf->X_f32[1];
	Q[2] = srckf->X_f32[2];
	Q[3] = srckf->X_f32[3];
}

#endif
