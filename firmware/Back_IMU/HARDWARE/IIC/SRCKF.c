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

#include "SRCKF.h"
#include "FastMath.h"
#include "Quaternion.h"

//////////////////////////////////////////////////////////////////////////
//all parameters below need to be tune
#define SRCKF_PQ_INITIAL 0.000001f
#define SRCKF_PW_INITIAL 0.000001f

#define SRCKF_QQ_INITIAL 0.0000045f
#define SRCKF_QW_INITIAL 0.0000025f

#define SRCKF_RA_INITIAL 0.07f
#define SRCKF_RM_INITIAL 0.105f
//////////////////////////////////////////////////////////////////////////
//
void SRCKF_New(SRCKF_Filter* srckf)
{
	float32_t kesi;
	arm_matrix_instance_f32 KesiPuls, KesiMinu;
	float32_t KesiPuls_f32[SRCKF_STATE_DIM * SRCKF_STATE_DIM], KesiMinus_f32[SRCKF_STATE_DIM * SRCKF_STATE_DIM];
	float32_t *S = srckf->S_f32;
	float32_t *SQ = srckf->SQ_f32;
	float32_t *SR = srckf->SR_f32;
	//////////////////////////////////////////////////////////////////////////
	//initialise weight
	srckf->W = 1.0f / (float32_t)SRCKF_CP_POINTS;
	arm_sqrt_f32(srckf->W, &srckf->SW);
	//initialise kesi
	//generate the cubature point
	kesi = (float32_t)SRCKF_STATE_DIM;
	arm_sqrt_f32(kesi, &kesi);
	arm_mat_init_f32(&KesiPuls, SRCKF_STATE_DIM, SRCKF_STATE_DIM, KesiPuls_f32);
	arm_mat_zero_f32(&KesiPuls);
	arm_mat_init_f32(&KesiMinu, SRCKF_STATE_DIM, SRCKF_STATE_DIM, KesiMinus_f32);
	arm_mat_zero_f32(&KesiMinu);
	arm_mat_identity_f32(&KesiPuls, kesi);
	arm_mat_identity_f32(&KesiMinu, -kesi);
	arm_mat_init_f32(&srckf->Kesi, SRCKF_STATE_DIM, SRCKF_CP_POINTS, srckf->Kesi_f32);
	arm_mat_setsubmatrix_f32(&srckf->Kesi, &KesiPuls, 0, 0);
	arm_mat_setsubmatrix_f32(&srckf->Kesi, &KesiMinu, 0, SRCKF_STATE_DIM);
	arm_mat_init_f32(&srckf->iKesi, SRCKF_STATE_DIM, 1, srckf->iKesi_f32);
	arm_mat_zero_f32(&srckf->iKesi);
	
	//initialise state covariance
	arm_mat_init_f32(&srckf->S, SRCKF_STATE_DIM, SRCKF_STATE_DIM, srckf->S_f32);
	arm_mat_zero_f32(&srckf->S);
	S[0] = S[8] = S[16] = S[24] = SRCKF_PQ_INITIAL;
	S[32] = S[40] = S[48] = SRCKF_PW_INITIAL;
	arm_mat_init_f32(&srckf->ST, SRCKF_STATE_DIM, SRCKF_STATE_DIM, srckf->ST_f32);
	//initialise measurement covariance
	arm_mat_init_f32(&srckf->SY, SRCKF_MEASUREMENT_DIM, SRCKF_MEASUREMENT_DIM, srckf->SY_f32);
	arm_mat_init_f32(&srckf->SYI, SRCKF_MEASUREMENT_DIM, SRCKF_MEASUREMENT_DIM, srckf->SYI_f32);
	arm_mat_init_f32(&srckf->SYT, SRCKF_MEASUREMENT_DIM, SRCKF_MEASUREMENT_DIM, srckf->SYT_f32);
	arm_mat_init_f32(&srckf->SYTI, SRCKF_MEASUREMENT_DIM, SRCKF_MEASUREMENT_DIM, srckf->SYTI_f32);
	arm_mat_init_f32(&srckf->PXY, SRCKF_STATE_DIM, SRCKF_MEASUREMENT_DIM, srckf->PXY_f32);
	arm_mat_init_f32(&srckf->tmpPXY, SRCKF_STATE_DIM, SRCKF_MEASUREMENT_DIM, srckf->tmpPXY_f32);
	//initialise SQ
	arm_mat_init_f32(&srckf->SQ, SRCKF_STATE_DIM, SRCKF_STATE_DIM, srckf->SQ_f32);
	arm_mat_zero_f32(&srckf->SQ);
	SQ[0] = SQ[8] = SQ[16] = SQ[24] = SRCKF_QQ_INITIAL;
	SQ[32] = SQ[40] = SQ[48] = SRCKF_QW_INITIAL;
	//initialise SR
	arm_mat_init_f32(&srckf->SR, SRCKF_MEASUREMENT_DIM, SRCKF_MEASUREMENT_DIM, srckf->SR_f32);
	arm_mat_zero_f32(&srckf->SR);
	SR[0] = SR[7] = SR[14] = SRCKF_RA_INITIAL;
	SR[21] = SR[28] = SR[35] = SRCKF_RM_INITIAL;
	//other stuff
	//cubature points
	arm_mat_init_f32(&srckf->XCP, SRCKF_STATE_DIM, SRCKF_CP_POINTS, srckf->XCP_f32);
	//propagated cubature points
	arm_mat_init_f32(&srckf->XPCP, SRCKF_STATE_DIM, SRCKF_CP_POINTS, srckf->XPCP_f32);
	arm_mat_init_f32(&srckf->YPCP, SRCKF_MEASUREMENT_DIM, SRCKF_CP_POINTS, srckf->YPCP_f32);
	arm_mat_init_f32(&srckf->XCPCM, SRCKF_STATE_DIM, SRCKF_CP_POINTS, srckf->XCPCM_f32);
	arm_mat_init_f32(&srckf->tmpXCPCM, SRCKF_STATE_DIM, SRCKF_CP_POINTS, srckf->tmpXCPCM_f32);
	arm_mat_init_f32(&srckf->YCPCM, SRCKF_MEASUREMENT_DIM, SRCKF_CP_POINTS, srckf->YCPCM_f32);
	arm_mat_init_f32(&srckf->YCPCMT, SRCKF_CP_POINTS, SRCKF_MEASUREMENT_DIM, srckf->YCPCMT_f32);
	
	arm_mat_init_f32(&srckf->XCM, SRCKF_STATE_DIM, SRCKF_CP_POINTS + SRCKF_STATE_DIM, srckf->XCM_f32);
	//initialise fill SQ
	arm_mat_setsubmatrix_f32(&srckf->XCM, &srckf->SQ, 0, SRCKF_CP_POINTS);
	
	arm_mat_init_f32(&srckf->YCM, SRCKF_MEASUREMENT_DIM, SRCKF_CP_POINTS + SRCKF_MEASUREMENT_DIM, srckf->YCM_f32);
	//initialise fill SR
	arm_mat_setsubmatrix_f32(&srckf->YCM, &srckf->SR, 0, SRCKF_CP_POINTS);
	
	arm_mat_init_f32(&srckf->XYCM, SRCKF_STATE_DIM, SRCKF_CP_POINTS + SRCKF_MEASUREMENT_DIM, srckf->XYCM_f32);
	//Kalman gain
	arm_mat_init_f32(&srckf->K, SRCKF_STATE_DIM, SRCKF_MEASUREMENT_DIM, srckf->K_f32);
	//////////////////////////////////////////////////////////////////////////
	//state vector
	arm_mat_init_f32(&srckf->X, SRCKF_STATE_DIM, 1, srckf->X_f32);
	arm_mat_zero_f32(&srckf->X);
	arm_mat_init_f32(&srckf->tmpX, SRCKF_STATE_DIM, 1, srckf->tmpX_f32);
	arm_mat_zero_f32(&srckf->tmpX);
	//measurement vector
	arm_mat_init_f32(&srckf->Y, SRCKF_MEASUREMENT_DIM, 1, srckf->Y_f32);
	arm_mat_zero_f32(&srckf->Y);
	arm_mat_init_f32(&srckf->tmpY, SRCKF_MEASUREMENT_DIM, 1, srckf->tmpY_f32);
	arm_mat_zero_f32(&srckf->tmpY);
}

void SRCKF_Init(SRCKF_Filter* srckf, float32_t *accel, float32_t *mag)
{	
	float32_t *X = srckf->X_f32;
	// local variables
	float32_t norma, normx, normy;

	//3x3 rotation matrix
	float32_t R[9];
	// place the un-normalized gravity and geomagnetic vectors into
	// the rotation matrix z and x axes
	R[2] = accel[0]; R[5] = accel[1]; R[8] = accel[2];
	R[0] = mag[0]; R[3] = mag[1]; R[6] = mag[2];
	// set y vector to vector product of z and x vectors
	R[1] = R[5] * R[6] - R[8] * R[3];
	R[4] = R[8] * R[0] - R[2] * R[6];
	R[7] = R[2] * R[3] - R[5] * R[0];
	// set x vector to vector product of y and z vectors
	R[0] = R[4] * R[8] - R[7] * R[5];
	R[3] = R[7] * R[2] - R[1] * R[8];
	R[6] = R[1] * R[5] - R[4] * R[2];
	// calculate the vector moduli invert
	norma = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	normx = FastSqrtI(R[0] * R[0] + R[3] * R[3] + R[6] * R[6]);
	normy = FastSqrtI(R[1] * R[1] + R[4] * R[4] + R[7] * R[7]);
	// normalize the rotation matrix
	// normalize x axis
	R[0] *= normx; R[3] *= normx; R[6] *= normx;
	// normalize y axis
	R[1] *= normy; R[4] *= normy; R[7] *= normy;
	// normalize z axis
	R[2] *= norma; R[5] *= norma; R[8] *= norma;	
	Quaternion_FromRotationMatrix(R, X);
}

void SRCKF_Update(SRCKF_Filter* srckf, float32_t *gyro, float32_t *accel, float32_t *mag, float32_t dt)
{	
	//////////////////////////////////////////////////////////////////////////
	float32_t q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	//
	float32_t hx, hy, hz;
	float32_t bx, bz;
	float32_t _2mx, _2my, _2mz;
	//
	int col;
	float32_t dQ[4];
	float32_t norm;
	float32_t *X = srckf->X_f32, *Y = srckf->Y_f32;
	float32_t *tmpX = srckf->tmpX_f32, *tmpY = srckf->tmpY_f32;
	float32_t *iKesi = srckf->iKesi_f32;
	
	dQ[0] = 0.0f;
	dQ[1] = (gyro[0] - X[4]);
	dQ[2] = (gyro[1] - X[5]);
	dQ[3] = (gyro[2] - X[6]);
	//////////////////////////////////////////////////////////////////////////
	//time update
	for(col = 0; col < SRCKF_CP_POINTS; col++){
		//evaluate the cubature points
		arm_mat_getcolumn_f32(&srckf->Kesi, iKesi, col);
		arm_mat_mult_f32(&srckf->S, &srckf->iKesi, &srckf->tmpX);
		arm_mat_add_f32(&srckf->tmpX, &srckf->X, &srckf->tmpX);
		arm_mat_setcolumn_f32(&srckf->XCP, tmpX, col);
		//evaluate the propagated cubature points
		Quaternion_RungeKutta4(tmpX, dQ, dt, 1);
		arm_mat_setcolumn_f32(&srckf->XPCP, tmpX, col);
	}
	//estimate the predicted state
	arm_mat_cumsum_f32(&srckf->XPCP, tmpX, X);
	arm_mat_scale_f32(&srckf->X, srckf->W, &srckf->X);
	//estimate the square-root factor of the predicted error covariance
	for(col = 0; col < SRCKF_CP_POINTS; col++){
		arm_mat_getcolumn_f32(&srckf->XPCP, tmpX, col);
		arm_mat_sub_f32(&srckf->tmpX, &srckf->X, &srckf->tmpX);
		arm_mat_scale_f32(&srckf->tmpX, srckf->SW, &srckf->tmpX);
		arm_mat_setcolumn_f32(&srckf->XCM, tmpX, col);
	}
	//XCM[XPCP, SQ], SQ fill already
	arm_mat_qr_decompositionT_f32(&srckf->XCM, &srckf->ST);
	arm_mat_trans_f32(&srckf->ST, &srckf->S);
	//////////////////////////////////////////////////////////////////////////
	//measurement update
	//normalize accel and magnetic
	norm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accel[0] *= norm;
	accel[1] *= norm;
	accel[2] *= norm;
	//////////////////////////////////////////////////////////////////////////
	norm = FastSqrtI(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	mag[0] *= norm;
	mag[1] *= norm;
	mag[2] *= norm;

	_2mx = 2.0f * mag[0];
	_2my = 2.0f * mag[1];
	_2mz = 2.0f * mag[2];
	for(col = 0; col < SRCKF_CP_POINTS; col++){
		//evaluate the cubature points
		arm_mat_getcolumn_f32(&srckf->Kesi, iKesi, col);
		arm_mat_mult_f32(&srckf->S, &srckf->iKesi, &srckf->tmpX);
		arm_mat_add_f32(&srckf->tmpX, &srckf->X, &srckf->tmpX);
		arm_mat_setcolumn_f32(&srckf->XCP, tmpX, col);
		//evaluate the propagated cubature points
		//auxiliary variables to avoid repeated arithmetic
		q0q0 = tmpX[0] * tmpX[0];
		q0q1 = tmpX[0] * tmpX[1];
		q0q2 = tmpX[0] * tmpX[2];
		q0q3 = tmpX[0] * tmpX[3];
		q1q1 = tmpX[1] * tmpX[1];
		q1q2 = tmpX[1] * tmpX[2];
		q1q3 = tmpX[1] * tmpX[3];
		q2q2 = tmpX[2] * tmpX[2];
		q2q3 = tmpX[2] * tmpX[3];
		q3q3 = tmpX[3] * tmpX[3];
		//reference direction of earth's magnetic field
		hx = _2mx * (0.5f - q2q2 - q3q3) + _2my * (q1q2 - q0q3) + _2mz *(q1q3 + q0q2);
		hy = _2mx * (q1q2 + q0q3) + _2my * (0.5f - q1q1 - q3q3) + _2mz * (q2q3 - q0q1);
		hz = _2mx * (q1q3 - q0q2) + _2my * (q2q3 + q0q1) + _2mz *(0.5f - q1q1 - q2q2);
		arm_sqrt_f32(hx * hx + hy * hy, &bx);
		bz = hz;
		
		tmpY[0] = 2.0f * (q1q3 - q0q2);
		tmpY[1] = 2.0f * (q2q3 + q0q1);
		tmpY[2] = -1.0f + 2.0f * (q0q0 + q3q3);
		tmpY[3] = bx * (1.0f - 2.0f * (q2q2 + q3q3)) + bz * ( 2.0f * (q1q3 - q0q2));
		tmpY[4] = bx * (2.0f * (q1q2 - q0q3)) + bz * (2.0f * (q2q3 + q0q1));
		tmpY[5] = bx * (2.0f * (q1q3 + q0q2)) + bz * (1.0f - 2.0f * (q1q1 + q2q2));
		arm_mat_setcolumn_f32(&srckf->YPCP, tmpY, col);
	}
	//estimate the predicted measurement
	arm_mat_cumsum_f32(&srckf->YPCP, tmpY, Y);
	arm_mat_scale_f32(&srckf->Y, srckf->W, &srckf->Y);
	//estimate the square-root of the innovation covariance matrix
	for(col = 0; col < SRCKF_CP_POINTS; col++){
		arm_mat_getcolumn_f32(&srckf->YPCP, tmpY, col);
		arm_mat_sub_f32(&srckf->tmpY, &srckf->Y, &srckf->tmpY);
		arm_mat_scale_f32(&srckf->tmpY, srckf->SW, &srckf->tmpY);
		arm_mat_setcolumn_f32(&srckf->YCPCM, tmpY, col);
		arm_mat_setcolumn_f32(&srckf->YCM, tmpY, col);
	}
	//YCM[YPCP, SR], SR fill already
	arm_mat_qr_decompositionT_f32(&srckf->YCM, &srckf->SYT);
	
	//estimate the cross-covariance matrix
	for(col = 0; col < SRCKF_CP_POINTS; col++){
		arm_mat_getcolumn_f32(&srckf->XCP, tmpX, col);
		arm_mat_sub_f32(&srckf->tmpX, &srckf->X, &srckf->tmpX);
		arm_mat_scale_f32(&srckf->tmpX, srckf->SW, &srckf->tmpX);
		arm_mat_setcolumn_f32(&srckf->XCPCM, tmpX, col);
	}
	arm_mat_trans_f32(&srckf->YCPCM, &srckf->YCPCMT);
	arm_mat_mult_f32(&srckf->XCPCM, &srckf->YCPCMT, &srckf->PXY);
	
	//estimate the kalman gain
	arm_mat_trans_f32(&srckf->SYT, &srckf->SY);
	arm_mat_inverse_f32(&srckf->SYT, &srckf->SYTI);
	arm_mat_inverse_f32(&srckf->SY, &srckf->SYI);
	arm_mat_mult_f32(&srckf->PXY, &srckf->SYTI, &srckf->tmpPXY);
	arm_mat_mult_f32(&srckf->tmpPXY, &srckf->SYI, &srckf->K);
	
	//estimate the updated state
	Y[0] = accel[0] - Y[0];
	Y[1] = accel[1] - Y[1];
	Y[2] = accel[2] - Y[2];
	Y[3] = mag[0] - Y[3];
	Y[4] = mag[1] - Y[4];
	Y[5] = mag[2] - Y[5];
	arm_mat_mult_f32(&srckf->K, &srckf->Y, &srckf->tmpX);
	arm_mat_add_f32(&srckf->X, &srckf->tmpX, &srckf->X);
	//normalize quaternion
	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;
	//estimate the square-root factor of the corresponding error covariance
	arm_mat_mult_f32(&srckf->K, &srckf->YCPCM, &srckf->tmpXCPCM);
	arm_mat_sub_f32(&srckf->XCPCM, &srckf->tmpXCPCM, &srckf->XCPCM);
	arm_mat_setsubmatrix_f32(&srckf->XYCM, &srckf->XCPCM, 0, 0);
	arm_mat_mult_f32(&srckf->K, &srckf->SR, &srckf->tmpPXY);
	arm_mat_setsubmatrix_f32(&srckf->XYCM, &srckf->tmpPXY, 0, SRCKF_CP_POINTS);
	arm_mat_qr_decompositionT_f32(&srckf->XYCM, &srckf->ST);
	arm_mat_trans_f32(&srckf->ST, &srckf->S);
}

void SRCKF_GetAngle(SRCKF_Filter* srckf, float32_t* rpy)
{
	float32_t R[3][3];
	float32_t *X = srckf->X_f32;
	//Z-Y-X
	R[0][0] = 2.0f * (X[0] * X[0] + X[1] * X[1]) - 1.0f;
	R[0][1] = 2.0f * (X[1] * X[2] + X[0] * X[3]);
	R[0][2] = 2.0f * (X[1] * X[3] - X[0] * X[2]);
	//R[1][0] = 2.0f * (X[1] * X[2] - X[0] * X[3]);
	//R[1][1] = 2.0f * (X[0] * X[0] + X[2] * X[2]) - 1.0f;
	R[1][2] = 2.0f * (X[2] * X[3] + X[0] * X[1]);
	//R[2][0] = 2.0f * (X[1] * X[3] + X[0] * X[2]);
	//R[2][1] = 2.0f * (X[2] * X[3] - X[0] * X[1]);
	R[2][2] = 2.0f * (X[0] * X[0] + X[3] * X[3]) - 1.0f;

	//roll
	rpy[0] = FastAtan2(R[1][2], R[2][2]);
	if (rpy[0] == SRCKF_PI)
		rpy[0] = -SRCKF_PI;
	//pitch
	if (R[0][2] >= 1.0f)
		rpy[1] = -SRCKF_HALFPI;
	else if (R[0][2] <= -1.0f)
		rpy[1] = SRCKF_HALFPI;
	else
		rpy[1] = FastAsin(-R[0][2]);
	//yaw
	rpy[2] = FastAtan2(R[0][1], R[0][0]);
	if (rpy[2] < 0.0f){
		rpy[2] += SRCKF_TWOPI;
	}
	if (rpy[2] > SRCKF_TWOPI){
		rpy[2] = 0.0f;
	}

	rpy[0] = SRCKF_TODEG(rpy[0]);
	rpy[1] = SRCKF_TODEG(rpy[1]);
	rpy[2] = SRCKF_TODEG(rpy[2]);
}
