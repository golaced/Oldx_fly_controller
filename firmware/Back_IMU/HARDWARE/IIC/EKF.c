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

#include "EKF.h"
#include "FastMath.h"
#include "Quaternion.h"

#define USE_4TH_RUNGE_KUTTA
//////////////////////////////////////////////////////////////////////////
//all parameters below need to be tune
#define EKF_PQ_INITIAL 0.000001f
#define EKF_PW_INITIAL 0.000010f

#define EKF_QQ_INITIAL 0.000045f
#define EKF_QW_INITIAL 0.00025f

#define EKF_RQ_INITIAL 0.000001f
#define EKF_RA_INITIAL 0.07f
#define EKF_RW_INITIAL 0.525f
#define EKF_RM_INITIAL 0.105f
//////////////////////////////////////////////////////////////////////////

void EKF_New(EKF_Filter* ekf)
{
	float32_t *H = ekf->H_f32;
	float32_t *P = ekf->P_f32;
	float32_t *Q = ekf->Q_f32;
	float32_t *R = ekf->R_f32;
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ekf->P, EKF_STATE_DIM, EKF_STATE_DIM, ekf->P_f32);
	arm_mat_zero_f32(&ekf->P);
	P[0] = P[8] = P[16] = P[24] = EKF_PQ_INITIAL;
	P[32] = P[40] = P[48] = EKF_PW_INITIAL;
	//
	arm_mat_init_f32(&ekf->Q, EKF_STATE_DIM, EKF_STATE_DIM, ekf->Q_f32);
	arm_mat_zero_f32(&ekf->Q);
	Q[0] = Q[8] = Q[16] = Q[24] = EKF_QQ_INITIAL;
	Q[32] = Q[40] = Q[48] = EKF_QW_INITIAL;
	//
	arm_mat_init_f32(&ekf->R, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, ekf->R_f32);
	arm_mat_zero_f32(&ekf->R);
	R[0] = R[14] = R[28] = R[42] = EKF_RQ_INITIAL;
	R[56] = R[70] = R[84] = EKF_RA_INITIAL;
	R[98] = R[112] = R[126] = EKF_RW_INITIAL;
	R[140] = R[154] = R[168] = EKF_RM_INITIAL;
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ekf->K, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, ekf->K_f32);
	arm_mat_init_f32(&ekf->KT, EKF_MEASUREMENT_DIM, EKF_STATE_DIM, ekf->KT_f32);
	arm_mat_init_f32(&ekf->S, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, ekf->S_f32);
	//
	arm_mat_init_f32(&ekf->F, EKF_STATE_DIM, EKF_STATE_DIM, ekf->F_f32);
	arm_mat_zero_f32(&ekf->F);
	arm_mat_identity_f32(&ekf->F, 1.0f);
	//
	arm_mat_init_f32(&ekf->FT, EKF_STATE_DIM, EKF_STATE_DIM, ekf->FT_f32);
	//
	arm_mat_init_f32(&ekf->H, EKF_MEASUREMENT_DIM, EKF_STATE_DIM, H);
	arm_mat_zero_f32(&ekf->H);
	H[0] = H[8] = H[16] = H[24] = 1.0f; //q row 0~3, col 0~3
	H[53] = H[61] = H[69] = 1.0f; //w row 7~9, col 4~6
	//
	arm_mat_init_f32(&ekf->HT, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, ekf->HT_f32);
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ekf->I, EKF_STATE_DIM, EKF_STATE_DIM, ekf->I_f32);
	arm_mat_zero_f32(&ekf->I);
	arm_mat_identity_f32(&ekf->I, 1.0f);
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ekf->X, EKF_STATE_DIM, 1, ekf->X_f32);
	arm_mat_zero_f32(&ekf->X);
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ekf->Y, EKF_MEASUREMENT_DIM, 1, ekf->Y_f32);
	arm_mat_zero_f32(&ekf->Y);
	//////////////////////////////////////////////////////////////////////////
	//
	arm_mat_init_f32(&ekf->tmpP, EKF_STATE_DIM, EKF_STATE_DIM, ekf->tmpP_f32);
	arm_mat_init_f32(&ekf->tmpX, EKF_STATE_DIM, 1, ekf->tmpX_f32);
	arm_mat_init_f32(&ekf->tmpYX, EKF_MEASUREMENT_DIM, EKF_STATE_DIM, ekf->tmpYX_f32);
	arm_mat_init_f32(&ekf->tmpXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, ekf->tmpXY_f32);
	arm_mat_init_f32(&ekf->tmpXX, EKF_STATE_DIM, EKF_STATE_DIM, ekf->tmpXX_f32);
	arm_mat_init_f32(&ekf->tmpXXT, EKF_STATE_DIM, EKF_STATE_DIM, ekf->tmpXXT_f32);
	arm_mat_init_f32(&ekf->tmpS, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, ekf->tmpS_f32);
	//////////////////////////////////////////////////////////////////////////
}

void EKF_Init(EKF_Filter* ekf, float32_t *q, float32_t *gyro)
{	
	float32_t *X = ekf->X_f32;
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

void EFK_Update(EKF_Filter* ekf, float32_t *q, float32_t *gyro, float32_t *accel, float32_t *mag, float32_t dt)
{
	float32_t norm;
	float32_t h[EKF_MEASUREMENT_DIM];

	float32_t halfdx, halfdy, halfdz;
	float32_t neghalfdx, neghalfdy, neghalfdz;
	float32_t halfdtq0, halfdtq1, neghalfdtq1, halfdtq2, neghalfdtq2, halfdtq3, neghalfdtq3;
	float32_t halfdt = 0.5f * dt;
#ifdef USE_4TH_RUNGE_KUTTA
	float32_t tmpW[4];
#endif
	//////////////////////////////////////////////////////////////////////////
	float32_t q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float32_t _2q0,_2q1,_2q2,_2q3;
	float32_t q0, q1, q2, q3;
	//
	float32_t hx, hy, hz;
	float32_t bx, bz;
	float32_t _2mx, _2my, _2mz;
	//
	float32_t *H = ekf->H_f32, *F = ekf->F_f32;
	float32_t *X = ekf->X_f32, *Y = ekf->Y_f32;
	//////////////////////////////////////////////////////////////////////////
	halfdx = halfdt * X[4];
	neghalfdx = -halfdx;
	halfdy = halfdt * X[5];
	neghalfdy = -halfdy;
	halfdz = halfdt * X[6];
	neghalfdz = -halfdz;

	//
	q0 = X[0];
	q1 = X[1];
	q2 = X[2];
	q3 = X[3];

	halfdtq0 = halfdt * q0;
	halfdtq1 = halfdt * q1;
	neghalfdtq1 = -halfdtq1;
	halfdtq2 = halfdt * q2;
	neghalfdtq2 = -halfdtq2;
	halfdtq3 = halfdt * q3;
	neghalfdtq3 = -halfdtq3;

	//F[0] = 1.0f;
	F[1] = neghalfdx;
	F[2] = neghalfdy;
	F[3] = neghalfdz;
	F[4] = neghalfdtq1;
	F[5] = neghalfdtq2;
	F[6] = neghalfdtq3;

	F[7] = halfdx;
	//F[8] = 1.0f;
	F[9] = neghalfdz;
	F[10] = halfdy;
	F[11] = halfdtq0;
	F[12] = halfdtq3;
	F[13] = neghalfdtq2;

	F[14] = halfdy;
	F[15] = halfdz;
	//F[16] = 1.0f;
	F[17] = neghalfdx;
	F[18] = neghalfdtq3;
	F[19] = halfdtq0;
	F[20] = neghalfdtq1;

	F[21] = halfdz;
	F[22] = neghalfdy;
	F[23] = halfdx;
	//F[24] = 1.0f;
	F[25] = halfdtq2;
	F[26] = neghalfdtq1;
	F[27] = halfdtq0;

	//model prediction
	//simple way, pay attention!!!
	//according to the actual gyroscope output
	//and coordinate system definition
#ifdef USE_4TH_RUNGE_KUTTA
	tmpW[0] = 0;
	tmpW[1] = X[4];
	tmpW[2] = X[5];
	tmpW[3] = X[6];
	Quaternion_RungeKutta4(X, tmpW, dt, 1);
#else
	X[0] = q0 - (halfdx * q1 + halfdy * q2 + halfdz * q3);
	X[1] = q1 + (halfdx * q0 + halfdy * q3 - halfdz * q2);
	X[2] = q2 - (halfdx * q3 - halfdy * q0 - halfdz * q1);
	X[3] = q3 + (halfdx * q2 - halfdy * q1 + halfdz * q0);
	//////////////////////////////////////////////////////////////////////////
	//Re-normalize Quaternion
	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;
#endif
	//X covariance matrix update based on model
	//P = F*P*F' + Q;
	arm_mat_trans_f32(&ekf->F, &ekf->FT);
	arm_mat_mult_f32(&ekf->F, &ekf->P, &ekf->tmpP);
	arm_mat_mult_f32(&ekf->tmpP, &ekf->FT, &ekf->P);
	arm_mat_add_f32(&ekf->P, &ekf->Q, &ekf->tmpP);

	//////////////////////////////////////////////////////////////////////////
	//model and measurement differences
	//normalize acc and mag measurements
	norm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accel[0] *= norm;
	accel[1] *= norm;
	accel[2] *= norm;
	//////////////////////////////////////////////////////////////////////////
	norm = FastSqrtI(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	mag[0] *= norm;
	mag[1] *= norm;
	mag[2] *= norm;

	//Auxiliary variables to avoid repeated arithmetic
	_2q0 = 2.0f * X[0];
	_2q1 = 2.0f * X[1];
	_2q2 = 2.0f * X[2];
	_2q3 = 2.0f * X[3];
	//
	q0q0 = X[0] * X[0];
	q0q1 = X[0] * X[1];
	q0q2 = X[0] * X[2];
	q0q3 = X[0] * X[3];
	q1q1 = X[1] * X[1];
	q1q2 = X[1] * X[2];
	q1q3 = X[1] * X[3];
	q2q2 = X[2] * X[2];
	q2q3 = X[2] * X[3];
	q3q3 = X[3] * X[3];

	_2mx = 2.0f * mag[0];
	_2my = 2.0f * mag[1];
	_2mz = 2.0f * mag[2];

	//Reference direction of Earth's magnetic field
	hx = _2mx * (0.5f - q2q2 - q3q3) + _2my * (q1q2 - q0q3) + _2mz *(q1q3 + q0q2);
	hy = _2mx * (q1q2 + q0q3) + _2my * (0.5f - q1q1 - q3q3) + _2mz * (q2q3 - q0q1);
	hz = _2mx * (q1q3 - q0q2) + _2my * (q2q3 + q0q1) + _2mz *(0.5f - q1q1 - q2q2);
	arm_sqrt_f32(hx * hx + hy * hy, &bx);
	bz = hz;

	h[0] = X[0];
	h[1] = X[1];
	h[2] = X[2];
	h[3] = X[3];

	h[4] = 2.0f * (q1q3 - q0q2);
	h[5] = 2.0f * (q2q3 + q0q1);
	h[6] = -1.0f + 2.0f * (q0q0 + q3q3);

	h[7] = X[4];
	h[8] = X[5];
	h[9] = X[6];

	h[10] = bx * (1.0f - 2.0f * (q2q2 + q3q3)) + bz * ( 2.0f * (q1q3 - q0q2));
	h[11] = bx * (2.0f * (q1q2 - q0q3)) + bz * (2.0f * (q2q3 + q0q1));
	h[12] = bx * (2.0f * (q1q3 + q0q2)) + bz * (1.0f - 2.0f * (q1q1 + q2q2));

	/////////////////////////////////////////////////////////////////////////
	//The H matrix maps the measurement to the states 13 x 7
	//row started from 0 to 12, col started from 0 to 6
	//row 4, col 0~3
	H[28] = -_2q2; H[29] = _2q3; H[30] = -_2q0; H[31] = _2q1;
	//row 5, col 0~3
	H[35] = _2q1; H[36] = _2q0; H[37] = _2q3; H[38] = _2q2;
	//row 6, col 0~3
	H[42] = _2q0; H[43] = -_2q1; H[44] = -_2q2;	H[45] = _2q3;
	//row 10, col 0~3
	H[70] = bx * _2q0 - bz * _2q2;
	H[71] = bx * _2q1 + bz * _2q3;
	H[72] = -bx * _2q2 - bz * _2q0;
	H[73] = bz * _2q1 - bx * _2q3;
	//row 11, col 0~3
	H[77] = bz * _2q1 - bx * _2q3;
	H[78] = bx * _2q2 + bz * _2q0;
	H[79] = bx * _2q1 + bz * _2q3;
	H[80] = bz * _2q2 - bx * _2q0;
	//row 12, col 0~3
	H[84] = bx * _2q2 + bz * _2q0;
	H[85] = bx * _2q3 - bz * _2q1;
	H[86] = bx * _2q0 - bz * _2q2;
	H[87] = bx * _2q1 + bz * _2q3;
	//
	//y = z - h;
	Y[0] = q[0] - h[0];
	Y[1] = q[1] - h[1];
	Y[2] = q[2] - h[2];
	Y[3] = q[3] - h[3];
	//
	Y[4] = accel[0] - h[4];
	Y[5] = accel[1] - h[5];
	Y[6] = accel[2] - h[6];
	Y[7] = gyro[0] - h[7];
	Y[8] = gyro[1] - h[8];
	Y[9] = gyro[2] - h[9];
	//////////////////////////////////////////////////////////////////////////
	Y[10] = mag[0] - h[10];
	Y[11] = mag[1] - h[11];
	Y[12] = mag[2] - h[12];

	//////////////////////////////////////////////////////////////////////////
	//Measurement covariance update
	//S = H*P*H' + R;
	arm_mat_trans_f32(&ekf->H, &ekf->HT);
	arm_mat_mult_f32(&ekf->H, &ekf->tmpP, &ekf->tmpYX);
	arm_mat_mult_f32(&ekf->tmpYX, &ekf->HT, &ekf->S);
	arm_mat_add_f32(&ekf->S, &ekf->R, &ekf->tmpS);

	//Calculate Kalman gain
	//K = P*H'/S;
	arm_mat_inverse_f32(&ekf->tmpS, &ekf->S);
	arm_mat_mult_f32(&ekf->tmpP, &ekf->HT, &ekf->tmpXY);
	arm_mat_mult_f32(&ekf->tmpXY, &ekf->S, &ekf->K);
	//Corrected model prediction
	//S = S + K*y;

	arm_mat_mult_f32(&ekf->K, &ekf->Y, &ekf->tmpX);
	arm_mat_add_f32(&ekf->X, &ekf->tmpX, &ekf->X);
	//normalize quaternion
	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;

	//Update state covariance with new knowledge
	//option: P = P - K*H*P or P = (I - K*H)*P*(I - K*H)' + K*R*K'
#if 0
	//P = P - K*H*P
	//simple but it can't ensure the matrix will be a positive definite matrix
	arm_mat_mult_f32(&ekf->K, &ekf->H, &ekf->tmpXX);
	arm_mat_mult_f32(&ekf->tmpXX, &ekf->tmpP, &ekf->P);
	arm_mat_sub_f32(&ekf->tmpP, &ekf->P, &ekf->P);
#else
	//P=(I - K*H)*P*(I - K*H)' + K*R*K'
	arm_mat_mult_f32(&ekf->K, &ekf->H, &ekf->tmpXX);
	arm_mat_sub_f32(&ekf->I, &ekf->tmpXX, &ekf->tmpXX);
	arm_mat_trans_f32(&ekf->tmpXX, &ekf->tmpXXT);
	arm_mat_mult_f32(&ekf->tmpXX, &ekf->tmpP, &ekf->P);
	arm_mat_mult_f32(&ekf->P, &ekf->tmpXXT, &ekf->tmpP);
	arm_mat_trans_f32(&ekf->K, &ekf->KT);
	arm_mat_mult_f32(&ekf->K, &ekf->R, &ekf->tmpXY);
	arm_mat_mult_f32(&ekf->tmpXY, &ekf->KT, &ekf->tmpXX);
	arm_mat_add_f32(&ekf->tmpP, &ekf->tmpXX, &ekf->P);
#endif
}

void EKF_GetAngle(EKF_Filter* ekf, float32_t* rpy)
{
	float32_t R[3][3];
	float32_t *X = ekf->X_f32;
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
	if (rpy[0] == EKF_PI)
		rpy[0] = -EKF_PI;
	//pitch
	if (R[0][2] >= 1.0f)
		rpy[1] = -EKF_HALFPI;
	else if (R[0][2] <= -1.0f)
		rpy[1] = EKF_HALFPI;
	else
		rpy[1] = FastAsin(-R[0][2]);
	//yaw
	rpy[2] = FastAtan2(R[0][1], R[0][0]);
	if (rpy[2] < 0.0f){
		rpy[2] += EKF_TWOPI;
	}
	if (rpy[2] > EKF_TWOPI){
		rpy[2] = 0.0f;
	}

	rpy[0] = EKF_TODEG(rpy[0]);
	rpy[1] = EKF_TODEG(rpy[1]);
	rpy[2] = EKF_TODEG(rpy[2]);
}
