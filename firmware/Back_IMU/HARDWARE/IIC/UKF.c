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

#include "UKF.h"
#include "FastMath.h"
#include "Quaternion.h"

#define USE_4TH_RUNGE_KUTTA
//////////////////////////////////////////////////////////////////////////
//all parameters below need to be tune
#define UKF_PQ_INITIAL 0.000001f
#define UKF_PW_INITIAL 0.000001f

#define UKF_QQ_INITIAL 0.0000045f
#define UKF_QW_INITIAL 0.0000025f

#define UKF_RQ_INITIAL 0.000001f
#define UKF_RA_INITIAL 0.07f
#define UKF_RW_INITIAL 0.0525f
#define UKF_RM_INITIAL 0.105f

#define UKF_alpha (1.0f)
#define UKF_beta (2.0f)
#define UKF_kappa (-1.0f)
//////////////////////////////////////////////////////////////////////////
//
static void UKF_GenerateSigmaPoints(UKF_Filter* ukf)
{
	int cols = 0;
	//state
	float32_t *X = ukf->X_f32;
	float32_t *tmpX = ukf->tmpX_f32;

	//need to tune!!!
	//covariance
	//c*chol(P)'
	arm_mat_chol_f32(&ukf->P);
	arm_mat_scale_f32(&ukf->P, ukf->gamma, &ukf->P);
	//////////////////////////////////////////////////////////////////////////
	//generate sigma points
	arm_mat_setcolumn_f32(&ukf->XSP, X, cols);
	for(cols = 1; cols <= UKF_STATE_DIM; cols++){
		arm_mat_getcolumn_f32(&ukf->P, tmpX, cols - 1);
		arm_mat_add_f32(&ukf->X, &ukf->tmpX, &ukf->tmpX);
		arm_mat_setcolumn_f32(&ukf->XSP, tmpX, cols);
	}
	for(cols = UKF_STATE_DIM + 1; cols < UKF_SP_POINTS; cols++){
		arm_mat_getcolumn_f32(&ukf->P, tmpX, cols - 8/*UKF_STATE_DIM + 1*/);
		arm_mat_sub_f32(&ukf->X, &ukf->tmpX, &ukf->tmpX);
		arm_mat_setcolumn_f32(&ukf->XSP, tmpX, cols);
	}
}

void UKF_New(UKF_Filter* ukf)
{
	float32_t *P = ukf->P_f32;
	float32_t *Q = ukf->Q_f32;
	float32_t *R = ukf->R_f32;
	float32_t *Wc = ukf->Wc_f32;

	//scaling factor
	float32_t lambda = UKF_alpha * UKF_alpha *((float32_t)UKF_STATE_DIM + UKF_kappa) - (float32_t)UKF_STATE_DIM;
	float32_t gamma = (float32_t)UKF_STATE_DIM + lambda;

	//weights for means
	ukf->Wm0 = lambda / gamma;
	ukf->Wmi = 0.5f / gamma;
	//weights for covariance
	arm_mat_init_f32(&ukf->Wc, UKF_SP_POINTS, UKF_SP_POINTS, ukf->Wc_f32);
	arm_mat_identity_f32(&ukf->Wc, ukf->Wmi);
	Wc[0] = ukf->Wm0 + (1.0f - UKF_alpha * UKF_alpha + UKF_beta);

	//scaling factor need to tune!
	arm_sqrt_f32(gamma, &ukf->gamma);
	//////////////////////////////////////////////////////////////////////////
	//initialise P
	arm_mat_init_f32(&ukf->P, UKF_STATE_DIM, UKF_STATE_DIM, ukf->P_f32);
	arm_mat_identity_f32(&ukf->P, 1.0f);
	P[0] = P[8] = P[16] = P[24] = UKF_PQ_INITIAL;
	P[32] = P[40] = P[48] = UKF_PW_INITIAL;

	arm_mat_init_f32(&ukf->PX, UKF_STATE_DIM, UKF_STATE_DIM, ukf->PX_f32);
	arm_mat_init_f32(&ukf->PY, UKF_MEASUREMENT_DIM, UKF_MEASUREMENT_DIM, ukf->PY_f32);
	arm_mat_init_f32(&ukf->tmpPY, UKF_MEASUREMENT_DIM, UKF_MEASUREMENT_DIM, ukf->tmpPY_f32);
	arm_mat_init_f32(&ukf->PXY, UKF_STATE_DIM, UKF_MEASUREMENT_DIM, ukf->PXY_f32);
	arm_mat_init_f32(&ukf->PXYT, UKF_MEASUREMENT_DIM, UKF_STATE_DIM, ukf->PXYT_f32);
	//initialise Q
	arm_mat_init_f32(&ukf->Q, UKF_STATE_DIM, UKF_STATE_DIM, ukf->Q_f32);
	Q[0] = Q[8] = Q[16] = Q[24] = UKF_QQ_INITIAL;
	Q[32] = Q[40] = Q[48] = UKF_QW_INITIAL;
	//initialise R
	arm_mat_init_f32(&ukf->R, UKF_MEASUREMENT_DIM, UKF_MEASUREMENT_DIM, ukf->R_f32);
	R[0] = R[14] = R[28] = R[42] = UKF_RQ_INITIAL;
	R[56] = R[70] = R[84] = UKF_RA_INITIAL;
	R[98] = R[112] = R[126] = UKF_RW_INITIAL;
	R[140] = R[154] = R[168] = UKF_RM_INITIAL;

	arm_mat_init_f32(&ukf->XSP, UKF_STATE_DIM, UKF_SP_POINTS, ukf->XSP_f32);
	arm_mat_init_f32(&ukf->tmpXSP, UKF_STATE_DIM, UKF_SP_POINTS, ukf->tmpXSP_f32);
	arm_mat_init_f32(&ukf->tmpXSPT, UKF_SP_POINTS, UKF_STATE_DIM, ukf->tmpXSPT_f32);
	arm_mat_init_f32(&ukf->tmpWcXSP, UKF_STATE_DIM, UKF_SP_POINTS, ukf->tmpWcXSP_f32);
	arm_mat_init_f32(&ukf->tmpWcYSP, UKF_MEASUREMENT_DIM, UKF_SP_POINTS, ukf->tmpWcYSP_f32);
	arm_mat_init_f32(&ukf->YSP, UKF_MEASUREMENT_DIM, UKF_SP_POINTS, ukf->YSP_f32);
	arm_mat_init_f32(&ukf->tmpYSP, UKF_MEASUREMENT_DIM, UKF_SP_POINTS, ukf->tmpYSP_f32);
	arm_mat_init_f32(&ukf->tmpYSPT, UKF_SP_POINTS, UKF_MEASUREMENT_DIM, ukf->tmpYSPT_f32);

	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ukf->K, UKF_STATE_DIM, UKF_MEASUREMENT_DIM, ukf->K_f32);
	arm_mat_init_f32(&ukf->KT, UKF_MEASUREMENT_DIM, UKF_STATE_DIM, ukf->KT_f32);
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ukf->X, UKF_STATE_DIM, 1, ukf->X_f32);
	arm_mat_zero_f32(&ukf->X);
	arm_mat_init_f32(&ukf->tmpX, UKF_STATE_DIM, 1, ukf->tmpX_f32);
	arm_mat_zero_f32(&ukf->tmpX);
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ukf->Y, UKF_MEASUREMENT_DIM, 1, ukf->Y_f32);
	arm_mat_zero_f32(&ukf->Y);
	arm_mat_init_f32(&ukf->tmpY, UKF_MEASUREMENT_DIM, 1, ukf->tmpY_f32);
	arm_mat_zero_f32(&ukf->tmpY);
	//////////////////////////////////////////////////////////////////////////
}

void UKF_Init(UKF_Filter* ukf, float32_t *q, float32_t *gyro)
{	
	float32_t *X = ukf->X_f32;
	float32_t norm;

	X[0] = q[0];
	X[1] = q[1];
	X[2] = q[2];
	X[3] = q[3];

	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;

	X[4] = gyro[0];
	X[5] = gyro[1];
	X[6] = gyro[2];
}

void UKF_Update(UKF_Filter* ukf, float32_t *q, float32_t *gyro, float32_t *accel, float32_t *mag, float32_t dt)
{
	int col, row;
	float32_t norm;
#ifndef USE_4TH_RUNGE_KUTTA
	float32_t halfdx, halfdy, halfdz;
	float32_t halfdt = 0.5f * dt;
#endif
	//////////////////////////////////////////////////////////////////////////
	float32_t q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	//
	float32_t hx, hy, hz;
	float32_t bx, bz;
	float32_t _2mx, _2my, _2mz;
	//
	float32_t *X = ukf->X_f32, *Y = ukf->Y_f32;
	float32_t *tmpX = ukf->tmpX_f32, *tmpY = ukf->tmpY_f32;
	float32_t tmpQ[4];
	//////////////////////////////////////////////////////////////////////////
	//calculate sigma points
	UKF_GenerateSigmaPoints(ukf);
	//time update
	//unscented transformation of process
	arm_mat_getcolumn_f32(&ukf->XSP, tmpX, 0);
#ifdef USE_4TH_RUNGE_KUTTA
	tmpQ[0] = 0;
	tmpQ[1] = tmpX[4];
	tmpQ[2] = tmpX[5];
	tmpQ[3] = tmpX[6];
	Quaternion_RungeKutta4(tmpX, tmpQ, dt, 1);
#else
	//
	halfdx = halfdt * tmpX[4];
	halfdy = halfdt * tmpX[5];
	halfdz = halfdt * tmpX[6];
	//
	tmpQ[0] = tmpX[0];
	tmpQ[1] = tmpX[1];
	tmpQ[2] = tmpX[2];
	tmpQ[3] = tmpX[3];
	//model prediction
	//simple way, pay attention!!!
	//according to the actual gyroscope output
	//and coordinate system definition
	tmpX[0] = tmpQ[0] + (halfdx * tmpQ[1] + halfdy * tmpQ[2] + halfdz * tmpQ[3]);
	tmpX[1] = tmpQ[1] - (halfdx * tmpQ[0] + halfdy * tmpQ[3] - halfdz * tmpQ[2]);
	tmpX[2] = tmpQ[2] + (halfdx * tmpQ[3] - halfdy * tmpQ[0] - halfdz * tmpQ[1]);
	tmpX[3] = tmpQ[3] - (halfdx * tmpQ[2] - halfdy * tmpQ[1] + halfdz * tmpQ[0]);
	//////////////////////////////////////////////////////////////////////////
	//Re-normalize Quaternion
	norm = FastSqrtI(tmpX[0] * tmpX[0] + tmpX[1] * tmpX[1] + tmpX[2] * tmpX[2] + tmpX[3] * tmpX[3]);
	tmpX[0] *= norm;
	tmpX[1] *= norm;
	tmpX[2] *= norm;
	tmpX[3] *= norm;
#endif
	//
	arm_mat_setcolumn_f32(&ukf->XSP, tmpX, 0);
	for(row = 0; row < UKF_STATE_DIM; row++){
		X[row] = ukf->Wm0 * tmpX[row];
	}
	for(col = 1; col < UKF_SP_POINTS; col++){
		//
		arm_mat_getcolumn_f32(&ukf->XSP, tmpX, col);
		//
#ifdef USE_4TH_RUNGE_KUTTA
		tmpQ[0] = 0;
		tmpQ[1] = tmpX[4];
		tmpQ[2] = tmpX[5];
		tmpQ[3] = tmpX[6];
		Quaternion_RungeKutta4(tmpX, tmpQ, dt, 1);
#else
		halfdx = halfdt * tmpX[4];
		halfdy = halfdt * tmpX[5];
		halfdz = halfdt * tmpX[6];
		//
		tmpQ[0] = tmpX[0];
		tmpQ[1] = tmpX[1];
		tmpQ[2] = tmpX[2];
		tmpQ[3] = tmpX[3];
		//model prediction
		//simple way, pay attention!!!
		//according to the actual gyroscope output
		//and coordinate system definition
		tmpX[0] = tmpQ[0] + (halfdx * tmpQ[1] + halfdy * tmpQ[2] + halfdz * tmpQ[3]);
		tmpX[1] = tmpQ[1] - (halfdx * tmpQ[0] + halfdy * tmpQ[3] - halfdz * tmpQ[2]);
		tmpX[2] = tmpQ[2] + (halfdx * tmpQ[3] - halfdy * tmpQ[0] - halfdz * tmpQ[1]);
		tmpX[3] = tmpQ[3] - (halfdx * tmpQ[2] - halfdy * tmpQ[1] + halfdz * tmpQ[0]);
		//////////////////////////////////////////////////////////////////////////
		//re-normalize quaternion
		norm = FastSqrtI(tmpX[0] * tmpX[0] + tmpX[1] * tmpX[1] + tmpX[2] * tmpX[2] + tmpX[3] * tmpX[3]);
		tmpX[0] *= norm;
		tmpX[1] *= norm;
		tmpX[2] *= norm;
		tmpX[3] *= norm;
#endif
		//
		arm_mat_setcolumn_f32(&ukf->XSP, tmpX, col);
		for(row = 0; row < UKF_STATE_DIM; row++){
			X[row] += ukf->Wmi * tmpX[row];
		}
	}
	for(col = 0; col < UKF_SP_POINTS; col++){
		arm_mat_getcolumn_f32(&ukf->XSP, tmpX, col);
		arm_mat_sub_f32(&ukf->tmpX, &ukf->X, &ukf->tmpX);
		arm_mat_setcolumn_f32(&ukf->tmpXSP, tmpX, col);
	}
	arm_mat_trans_f32(&ukf->tmpXSP, &ukf->tmpXSPT);
	arm_mat_mult_f32(&ukf->tmpXSP, &ukf->Wc, &ukf->tmpWcXSP);
	arm_mat_mult_f32(&ukf->tmpWcXSP, &ukf->tmpXSPT, &ukf->PX);
	arm_mat_add_f32(&ukf->PX, &ukf->Q, &ukf->PX);
	//////////////////////////////////////////////////////////////////////////
	//recalculate sigma points
	//UKF_GenerateSigmaPoints(ukf);

	//measurement update
	//unscented transformation of measurments
	//////////////////////////////////////////////////////////////////////////
	//Normalize accel and mag
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

	arm_mat_getcolumn_f32(&ukf->XSP, tmpX, 0);
	//Auxiliary variables to avoid repeated arithmetic
	//
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

	//Reference direction of Earth's magnetic field
	hx = _2mx * (0.5f - q2q2 - q3q3) + _2my * (q1q2 - q0q3) + _2mz *(q1q3 + q0q2);
	hy = _2mx * (q1q2 + q0q3) + _2my * (0.5f - q1q1 - q3q3) + _2mz * (q2q3 - q0q1);
	hz = _2mx * (q1q3 - q0q2) + _2my * (q2q3 + q0q1) + _2mz *(0.5f - q1q1 - q2q2);
	arm_sqrt_f32(hx * hx + hy * hy, &bx);
	bz = hz;

	tmpY[0] = tmpX[0];
	tmpY[1] = tmpX[1];
	tmpY[2] = tmpX[2];
	tmpY[3] = tmpX[3];
	tmpY[4] = 2.0f * (q1q3 - q0q2);
	tmpY[5] = 2.0f * (q2q3 + q0q1);
	tmpY[6] = -1.0f + 2.0f * (q0q0 + q3q3);
	tmpY[7] = tmpX[4];
	tmpY[8] = tmpX[5];
	tmpY[9] = tmpX[6];
	tmpY[10] = bx * (1.0f - 2.0f * (q2q2 + q3q3)) + bz * ( 2.0f * (q1q3 - q0q2));
	tmpY[11] = bx * (2.0f * (q1q2 - q0q3)) + bz * (2.0f * (q2q3 + q0q1));
	tmpY[12] = bx * (2.0f * (q1q3 + q0q2)) + bz * (1.0f - 2.0f * (q1q1 + q2q2));
	arm_mat_setcolumn_f32(&ukf->YSP, tmpY, 0);
	for(row = 0; row < UKF_MEASUREMENT_DIM; row++){
		Y[row] = ukf->Wm0 * tmpY[row];
	}

	for(col = 1; col < UKF_SP_POINTS; col++){
		arm_mat_getcolumn_f32(&ukf->XSP, tmpX, col);
		//Auxiliary variables to avoid repeated arithmetic
		//
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

		//Reference direction of Earth's magnetic field
		hx = _2mx * (0.5f - q2q2 - q3q3) + _2my * (q1q2 - q0q3) + _2mz *(q1q3 + q0q2);
		hy = _2mx * (q1q2 + q0q3) + _2my * (0.5f - q1q1 - q3q3) + _2mz * (q2q3 - q0q1);
		hz = _2mx * (q1q3 - q0q2) + _2my * (q2q3 + q0q1) + _2mz *(0.5f - q1q1 - q2q2);
		arm_sqrt_f32(hx * hx + hy * hy, &bx);
		bz = hz;

		tmpY[0] = tmpX[0];
		tmpY[1] = tmpX[1];
		tmpY[2] = tmpX[2];
		tmpY[3] = tmpX[3];
		tmpY[4] = 2.0f * (q1q3 - q0q2);
		tmpY[5] = 2.0f * (q2q3 + q0q1);
		tmpY[6] = -1.0f + 2.0f * (q0q0 + q3q3);
		tmpY[7] = tmpX[4];
		tmpY[8] = tmpX[5];
		tmpY[9] = tmpX[6];
		tmpY[10] = bx * (1.0f - 2.0f * (q2q2 + q3q3)) + bz * ( 2.0f * (q1q3 - q0q2));
		tmpY[11] = bx * (2.0f * (q1q2 - q0q3)) + bz * (2.0f * (q2q3 + q0q1));
		tmpY[12] = bx * (2.0f * (q1q3 + q0q2)) + bz * (1.0f - 2.0f * (q1q1 + q2q2));
		arm_mat_setcolumn_f32(&ukf->YSP, tmpY, col);
		for(row = 0; row < UKF_MEASUREMENT_DIM; row++){
			Y[row] += ukf->Wmi * tmpY[row];
		}
	}
	for(col = 0; col < UKF_SP_POINTS; col++){
		arm_mat_getcolumn_f32(&ukf->YSP, tmpY, col);
		arm_mat_sub_f32(&ukf->tmpY, &ukf->Y, &ukf->tmpY);
		arm_mat_setcolumn_f32(&ukf->tmpYSP, tmpY, col);
	}
	arm_mat_trans_f32(&ukf->tmpYSP, &ukf->tmpYSPT);
	arm_mat_mult_f32(&ukf->tmpYSP, &ukf->Wc, &ukf->tmpWcYSP);
	arm_mat_mult_f32(&ukf->tmpWcYSP, &ukf->tmpYSPT, &ukf->PY);
	arm_mat_add_f32(&ukf->PY, &ukf->R, &ukf->PY);
	//transformed cross-covariance
	arm_mat_mult_f32(&ukf->tmpWcXSP, &ukf->tmpYSPT, &ukf->PXY);
	//calculate kalman gate
	//K = PXY * inv(PY);
	arm_mat_inverse_f32(&ukf->PY, &ukf->tmpPY);
	arm_mat_mult_f32(&ukf->PXY, &ukf->tmpPY, &ukf->K);
	//state update
	//X = X + K*(z - Y);
	Y[0] = q[0] - Y[0];
	Y[1] = q[1] - Y[1];
	Y[2] = q[2] - Y[2];
	Y[3] = q[3] - Y[3];
	//
	Y[4] = accel[0] - Y[4];
	Y[5] = accel[1] - Y[5];
	Y[6] = accel[2] - Y[6];
	Y[7] = gyro[0] - Y[7];
	Y[8] = gyro[1] - Y[8];
	Y[9] = gyro[2] - Y[9];
	//////////////////////////////////////////////////////////////////////////
	Y[10] = mag[0] - Y[10];
	Y[11] = mag[1] - Y[11];
	Y[12] = mag[2] - Y[12];

	arm_mat_mult_f32(&ukf->K, &ukf->Y, &ukf->tmpX);
	arm_mat_add_f32(&ukf->X, &ukf->tmpX, &ukf->X);
	//normalize quaternion
	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;

	//covariance update
#if 0
	//the default tuning parameters don't work properly
	//so that use the simple update method below
	//original update method
	//P = PX - K * PY * K'
	arm_mat_trans_f32(&ukf->K, &ukf->KT);
	arm_mat_mult_f32(&ukf->K, &ukf->PY, &ukf->PXY);
	arm_mat_mult_f32(&ukf->PXY, &ukf->KT, &ukf->P);
	arm_mat_sub_f32(&ukf->PX, &ukf->P, &ukf->P);
#else
	//must tune the P,Q,R
	//simple update method
	//P = PX - K * PXY'
	arm_mat_trans_f32(&ukf->PXY, &ukf->PXYT);
	arm_mat_mult_f32(&ukf->K, &ukf->PXYT, &ukf->P);
	arm_mat_sub_f32(&ukf->PX, &ukf->P, &ukf->P);
#endif
}

void UKF_GetAngle(UKF_Filter* ukf, float32_t* rpy)
{
	float32_t R[3][3];
	float32_t *X = ukf->X_f32;
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
	if (rpy[0] == UKF_PI)
		rpy[0] = -UKF_PI;
	//pitch
	if (R[0][2] >= 1.0f)
		rpy[1] = -UKF_HALFPI;
	else if (R[0][2] <= -1.0f)
		rpy[1] = UKF_HALFPI;
	else
		rpy[1] = FastAsin(-R[0][2]);
	//yaw
	rpy[2] = FastAtan2(R[0][1], R[0][0]);
	if (rpy[2] < 0.0f){
		rpy[2] += UKF_TWOPI;
	}
	if (rpy[2] > UKF_TWOPI){
		rpy[2] = 0.0f;
	}

	rpy[0] = UKF_TODEG(rpy[0]);
	rpy[1] = UKF_TODEG(rpy[1]);
	rpy[2] = UKF_TODEG(rpy[2]);
}
