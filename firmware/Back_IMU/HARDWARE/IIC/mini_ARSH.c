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

#include "FastMath.h"
#include "Quaternion.h"
#include "miniARSH.h"
#include "miniMatrix.h"
//////////////////////////////////////////////////////////////////////////
//
//all parameters below need to be tune
#define EKF_PQ_INITIAL 0.001f
#define EKF_PWB_INITIAL 0.001f

#define EKF_QQ_INITIAL 0.05f
#define EKF_QWB_INITIAL 0.0000005f

#define EKF_RA_INITIAL 0.005346f
#define EKF_RM_INITIAL 0.005346f
//////////////////////////////////////////////////////////////////////////
//
#define UPDATE_P_COMPLICATED

#ifdef UPDATE_P_COMPLICATED
static float I_ARSH[EKF_STATE_DIM * EKF_STATE_DIM] = {
	1.0f, 0, 0, 0, 0, 0, 0,
	0, 1.0f, 0, 0, 0, 0, 0,
	0, 0, 1.0f, 0, 0, 0, 0,
	0, 0, 0, 1.0f, 0, 0, 0,
	0, 0, 0, 0, 1.0f, 0, 0,
	0, 0, 0, 0, 0, 1.0f, 0,
	0, 0, 0, 0, 0, 0, 1.0f,
};
#endif

static float P_ARSH[EKF_STATE_DIM * EKF_STATE_DIM] = {
	EKF_PQ_INITIAL, 0, 0, 0, 0, 0, 0,
	0, EKF_PQ_INITIAL, 0, 0, 0, 0, 0,
	0, 0, EKF_PQ_INITIAL, 0, 0, 0, 0,
	0, 0, 0, EKF_PQ_INITIAL, 0, 0, 0,
	0, 0, 0, 0, EKF_PWB_INITIAL, 0, 0,
	0, 0, 0, 0, 0, EKF_PWB_INITIAL, 0,
	0, 0, 0, 0, 0, 0, EKF_PWB_INITIAL,
};

static float Q_ARSH[EKF_STATE_DIM * EKF_STATE_DIM] = {
	EKF_QQ_INITIAL, 0, 0, 0, 0, 0, 0,
	0, EKF_QQ_INITIAL, 0, 0, 0, 0, 0,
	0, 0, EKF_QQ_INITIAL, 0, 0, 0, 0,
	0, 0, 0, EKF_QQ_INITIAL, 0, 0, 0,
	0, 0, 0, 0, EKF_QWB_INITIAL, 0, 0,
	0, 0, 0, 0, 0, EKF_QWB_INITIAL, 0,
	0, 0, 0, 0, 0, 0, EKF_QWB_INITIAL,
};

static float R_ARSH[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {
	EKF_RA_INITIAL, 0, 0, 0, 0, 0,
	0, EKF_RA_INITIAL, 0, 0, 0, 0,
	0, 0, EKF_RA_INITIAL, 0, 0, 0,
	0, 0, 0, EKF_RM_INITIAL, 0, 0,
	0, 0, 0, 0, EKF_RM_INITIAL, 0,
	0, 0, 0, 0, 0, EKF_RM_INITIAL,
};

static float F_ARSH[EKF_STATE_DIM * EKF_STATE_DIM] = {
	1.0f, 0, 0, 0, 0, 0, 0,
	0, 1.0f, 0, 0, 0, 0, 0,
	0, 0, 1.0f, 0, 0, 0, 0,
	0, 0, 0, 1.0f, 0, 0, 0,
	0, 0, 0, 0, 1.0f, 0, 0,
	0, 0, 0, 0, 0, 1.0f, 0,
	0, 0, 0, 0, 0, 0, 1.0f,
};

static float H_ARSH[EKF_MEASUREMENT_DIM * EKF_STATE_DIM] = {
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
};

//state
static float X_ARSH[EKF_STATE_DIM];
static float KY[EKF_STATE_DIM];
//measurement
static float Y_ARSH[EKF_MEASUREMENT_DIM];
//
static float CBn[9];
//
static float PX[EKF_STATE_DIM * EKF_STATE_DIM];
static float PXX[EKF_STATE_DIM * EKF_STATE_DIM];
static float PXY[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float K_ARSH[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float S_ARSH[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];

static void Calcultate_RotationMatrix(float *accel, float *mag, float *R_ARSH)
{
	// local variables
	float norm, fmodx, fmody;
	// place the un-normalized gravity and geomagnetic vectors into
	// the rotation matrix z and X_ARSH axes
	R_ARSH[2] = accel[0]; R_ARSH[5] = accel[1]; R_ARSH[8] = accel[2];
	R_ARSH[0] = mag[0]; R_ARSH[3] = mag[1]; R_ARSH[6] = mag[2];
	// set Y_ARSH vector to vector product of z and X_ARSH vectors
	R_ARSH[1] = R_ARSH[5] * R_ARSH[6] - R_ARSH[8] * R_ARSH[3];
	R_ARSH[4] = R_ARSH[8] * R_ARSH[0] - R_ARSH[2] * R_ARSH[6];
	R_ARSH[7] = R_ARSH[2] * R_ARSH[3] - R_ARSH[5] * R_ARSH[0];
	// set X_ARSH vector to vector product of Y_ARSH and z vectors
	R_ARSH[0] = R_ARSH[4] * R_ARSH[8] - R_ARSH[7] * R_ARSH[5];
	R_ARSH[3] = R_ARSH[7] * R_ARSH[2] - R_ARSH[1] * R_ARSH[8];
	R_ARSH[6] = R_ARSH[1] * R_ARSH[5] - R_ARSH[4] * R_ARSH[2];
	// calculate the vector moduli invert
	norm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	fmodx = FastSqrtI(R_ARSH[0] * R_ARSH[0] + R_ARSH[3] * R_ARSH[3] + R_ARSH[6] * R_ARSH[6]);
	fmody = FastSqrtI(R_ARSH[1] * R_ARSH[1] + R_ARSH[4] * R_ARSH[4] + R_ARSH[7] * R_ARSH[7]);
	// normalize the rotation matrix
	// normalize X_ARSH axis
	R_ARSH[0] *= fmodx; R_ARSH[3] *= fmodx; R_ARSH[6] *= fmodx;
	// normalize Y_ARSH axis
	R_ARSH[1] *= fmody; R_ARSH[4] *= fmody; R_ARSH[7] *= fmody;
	// normalize z axis
	R_ARSH[2] *= norm; R_ARSH[5] *= norm; R_ARSH[8] *= norm;
}

void EKF_AHRSInit(float *accel, float *mag)
{
	//3x3 rotation matrix
	float R_ARSH[9];
	
	Calcultate_RotationMatrix(accel, mag, R_ARSH);
	Quaternion_FromRotationMatrix(R_ARSH, X_ARSH);
}

void EKF_AHRSUpdate(float *gyro, float *accel, float *mag, float dt)
{
	float norm;
	float halfdx, halfdy, halfdz;
	float neghalfdx, neghalfdy, neghalfdz;
	float halfdtq0, neghalfdtq0, halfdtq1, neghalfdtq1,
		halfdtq2, neghalfdtq2, halfdtq3, neghalfdtq3;
	float halfdt = 0.5f * dt;
	//////////////////////////////////////////////////////////////////////////
	float _2q0,_2q1,_2q2,_2q3;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float q0, q1, q2, q3;
	float _2mx, _2my, _2mz;
	float hx, hy, hz;
	float bx, bz;
	//
	float SI[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {0};
	//////////////////////////////////////////////////////////////////////////
	halfdx = halfdt * (gyro[0] - X_ARSH[4]);
	halfdy = halfdt * (gyro[1] - X_ARSH[5]);
	halfdz = halfdt * (gyro[2] - X_ARSH[6]);
	neghalfdx = -halfdx; neghalfdy = -halfdy; neghalfdz = -halfdz;
	//
	q0 = X_ARSH[0]; q1 = X_ARSH[1]; q2 = X_ARSH[2]; q3 = X_ARSH[3];

	//////////////////////////////////////////////////////////////////////////
	//Extended Kalman Filter: Prediction Step
	//state time propagation
	//Update Quaternion with the new gyroscope measurements
	X_ARSH[0] = q0 - halfdx * q1 - halfdy * q2 - halfdz * q3;
	X_ARSH[1] = q1 + halfdx * q0 - halfdy * q3 + halfdz * q2;
	X_ARSH[2] = q2 + halfdx * q3 + halfdy * q0 - halfdz * q1;
	X_ARSH[3] = q3 - halfdx * q2 + halfdy * q1 + halfdz * q0;

	//normalize quaternion
	norm = FastSqrtI(X_ARSH[0] * X_ARSH[0] + X_ARSH[1] * X_ARSH[1] + X_ARSH[2] * X_ARSH[2] + X_ARSH[3] * X_ARSH[3]);
	X_ARSH[0] *= norm;
	X_ARSH[1] *= norm;
	X_ARSH[2] *= norm;
	X_ARSH[3] *= norm;

	//populate F_ARSH jacobian
	halfdtq0 = halfdt * q0; halfdtq1 = halfdt * q1; halfdtq2 = halfdt * q2; halfdtq3 = halfdt * q3;
	neghalfdtq0 = -halfdtq0; neghalfdtq1 = -halfdtq1; neghalfdtq2 = -halfdtq2; neghalfdtq3 = -halfdtq3;

	/* F_ARSH[0] = 1.0f; */ F_ARSH[1] = neghalfdx; F_ARSH[2] = neghalfdy; F_ARSH[3] = neghalfdz; F_ARSH[4] = halfdtq1; F_ARSH[5] = halfdtq2; F_ARSH[6] = halfdtq3;
	F_ARSH[7] = halfdx; /* F_ARSH[8] = 1.0f; */ F_ARSH[9] = halfdz;	F_ARSH[10] = neghalfdy; F_ARSH[11] = neghalfdtq0; F_ARSH[12] = halfdtq3; F_ARSH[13] = neghalfdtq2;
	F_ARSH[14] = halfdy;	F_ARSH[15] = neghalfdz;	/* F_ARSH[16] = 1.0f; */ F_ARSH[17] = halfdx; F_ARSH[18] = neghalfdtq3; F_ARSH[19] = neghalfdtq0; F_ARSH[20] = halfdtq1;
	F_ARSH[21] = halfdz; F_ARSH[22] = halfdy; F_ARSH[23] = neghalfdx; /* F_ARSH[24] = 1.0f; */ F_ARSH[25] = halfdtq2; F_ARSH[26] = neghalfdtq1; F_ARSH[27] = neghalfdtq0;

	//covariance time propagation
	//P_ARSH = F_ARSH*P_ARSH*F_ARSH' + Q_ARSH;
	Matrix_Multiply(F_ARSH, EKF_STATE_DIM, EKF_STATE_DIM, P_ARSH, EKF_STATE_DIM, PX);
	Matrix_Multiply_With_Transpose(PX, EKF_STATE_DIM, EKF_STATE_DIM, F_ARSH, EKF_STATE_DIM, P_ARSH);
	Maxtrix_Add(P_ARSH, EKF_STATE_DIM, EKF_STATE_DIM, Q_ARSH, P_ARSH);

	//////////////////////////////////////////////////////////////////////////
	//measurement update
	//normalize accel and magnetic
	norm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accel[0] *= norm;
	accel[1] *= norm;
	accel[2] *= norm;
	norm = FastSqrtI(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	mag[0] *= norm;
	mag[1] *= norm;
	mag[2] *= norm;
	
	//Reference field calculation	
	//auxiliary variables to avoid repeated arithmetic
	_2q0 = 2.0f * X_ARSH[0]; _2q1 = 2.0f * X_ARSH[1]; _2q2 = 2.0f * X_ARSH[2]; _2q3 = 2.0f * X_ARSH[3];
	//
	q0q0 = X_ARSH[0] * X_ARSH[0]; q0q1 = X_ARSH[0] * X_ARSH[1]; q0q2 = X_ARSH[0] * X_ARSH[2]; q0q3 = X_ARSH[0] * X_ARSH[3];
	q1q1 = X_ARSH[1] * X_ARSH[1]; q1q2 = X_ARSH[1] * X_ARSH[2]; q1q3 = X_ARSH[1] * X_ARSH[3];
	q2q2 = X_ARSH[2] * X_ARSH[2]; q2q3 = X_ARSH[2] * X_ARSH[3];
	q3q3 = X_ARSH[3] * X_ARSH[3];

	_2mx = 2.0f * mag[0]; _2my = 2.0f * mag[1]; _2mz = 2.0f * mag[2];

	hx = _2mx * (0.5f - q2q2 - q3q3) + _2my * (q1q2 - q0q3) + _2mz *(q1q3 + q0q2);
	hy = _2mx * (q1q2 + q0q3) + _2my * (0.5f - q1q1 - q3q3) + _2mz * (q2q3 - q0q1);
	hz = _2mx * (q1q3 - q0q2) + _2my * (q2q3 + q0q1) + _2mz *(0.5f - q1q1 - q2q2);
	bx = FastSqrt(hx * hx + hy * hy);
	bz = hz;
	//
	Y_ARSH[0] = -2.0f * (q1q3 - q0q2);
	Y_ARSH[1] = -2.0f * (q2q3 + q0q1);
	Y_ARSH[2] = 1.0f - 2.0f * (q0q0 + q3q3);
	Y_ARSH[3] = bx * (1.0f - 2.0f * (q2q2 + q3q3)) + bz * ( 2.0f * (q1q3 - q0q2));
	Y_ARSH[4] = bx * (2.0f * (q1q2 - q0q3)) + bz * (2.0f * (q2q3 + q0q1));
	Y_ARSH[5] = bx * (2.0f * (q1q3 + q0q2)) + bz * (1.0f - 2.0f * (q1q1 + q2q2));
	
	Y_ARSH[0] = accel[0] - Y_ARSH[0];
	Y_ARSH[1] = accel[1] - Y_ARSH[1];
	Y_ARSH[2] = accel[2] - Y_ARSH[2];
	Y_ARSH[3] = mag[0] - Y_ARSH[3];
	Y_ARSH[4] = mag[1] - Y_ARSH[4];
	Y_ARSH[5] = mag[2] - Y_ARSH[5];
	
	//populate H_ARSH jacobian
	H_ARSH[0] = _2q2; H_ARSH[1] = -_2q3; H_ARSH[2] = _2q0; H_ARSH[3] = -_2q1;
	H_ARSH[7] = -_2q1; H_ARSH[8] = -_2q0; H_ARSH[9] = -_2q3; H_ARSH[10] = -_2q2;
	H_ARSH[14] = -_2q0; H_ARSH[15] = _2q1; H_ARSH[16] = _2q2; H_ARSH[17] = -_2q3;
	
	H_ARSH[21] = bx * _2q0 - bz * _2q2; H_ARSH[22] = bx * _2q1 + bz * _2q3; H_ARSH[23] = -bx * _2q2 - bz * _2q0; H_ARSH[24] = bz * _2q1 - bx * _2q3;
	H_ARSH[28] = bz * _2q1 - bx * _2q3; H_ARSH[29] = bx * _2q2 + bz * _2q0;	 H_ARSH[30] = bx * _2q1 + bz * _2q3; H_ARSH[31] = bz * _2q2 - bx * _2q0;
	H_ARSH[35] = bx * _2q2 + bz * _2q0; H_ARSH[36] = bx * _2q3 - bz * _2q1; H_ARSH[37] = bx * _2q0 - bz * _2q2; H_ARSH[38] = bx * _2q1 + bz * _2q3;
	
	//kalman gain calculation
	//K_ARSH = P_ARSH * H_ARSH' / (R_ARSH + H_ARSH * P_ARSH * H_ARSH')
	//acceleration of gravity
	Matrix_Multiply_With_Transpose(P_ARSH, EKF_STATE_DIM, EKF_STATE_DIM, H_ARSH, EKF_MEASUREMENT_DIM, PXY);
	Matrix_Multiply(H_ARSH, EKF_MEASUREMENT_DIM, EKF_STATE_DIM, PXY, EKF_MEASUREMENT_DIM, S_ARSH);
	Maxtrix_Add(S_ARSH, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, R_ARSH, S_ARSH);
	Matrix_Inverse(S_ARSH, EKF_MEASUREMENT_DIM, SI);
	Matrix_Multiply(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, SI, EKF_MEASUREMENT_DIM, K_ARSH);

	//update state vector
	//X_ARSH = X_ARSH + K_ARSH * Y_ARSH;
	Matrix_Multiply(K_ARSH, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, Y_ARSH, 1, KY);
	Maxtrix_Add(X_ARSH, EKF_STATE_DIM, 1, KY, X_ARSH);
	
	//normalize quaternion
	norm = FastSqrtI(X_ARSH[0] * X_ARSH[0] + X_ARSH[1] * X_ARSH[1] + X_ARSH[2] * X_ARSH[2] + X_ARSH[3] * X_ARSH[3]);
	X_ARSH[0] *= norm;
	X_ARSH[1] *= norm;
	X_ARSH[2] *= norm;
	X_ARSH[3] *= norm;

	//covariance estimate update
	//P_ARSH = (I_ARSH - K_ARSH * H_ARSH) * P_ARSH
	//P_ARSH = P_ARSH - K_ARSH * H_ARSH * P_ARSH
	//or
	//P_ARSH=(I_ARSH - K_ARSH*H_ARSH)*P_ARSH*(I_ARSH - K_ARSH*H_ARSH)' + K_ARSH*R_ARSH*K_ARSH'
#ifndef UPDATE_P_COMPLICATED
	Matrix_Multiply(K_ARSH, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, H_ARSH, EKF_STATE_DIM, PX);
	Matrix_Multiply(PX, EKF_STATE_DIM, EKF_STATE_DIM, P_ARSH, EKF_STATE_DIM, PXX);
	Maxtrix_Add(P_ARSH, EKF_STATE_DIM, EKF_STATE_DIM, PXX, P_ARSH);
#else
	Matrix_Multiply(K_ARSH, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, H_ARSH, EKF_STATE_DIM, PX);
	Maxtrix_Sub(I_ARSH, EKF_STATE_DIM, EKF_STATE_DIM, PX, PX);
	Matrix_Multiply(PX, EKF_STATE_DIM, EKF_STATE_DIM, P_ARSH, EKF_STATE_DIM, PXX);
	Matrix_Multiply_With_Transpose(PXX, EKF_STATE_DIM, EKF_STATE_DIM, PX, EKF_STATE_DIM, P_ARSH);
	Matrix_Multiply(K_ARSH, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, R_ARSH, EKF_MEASUREMENT_DIM, PXY);
	Matrix_Multiply_With_Transpose(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, K_ARSH, EKF_STATE_DIM, PX);
	Maxtrix_Add(P_ARSH, EKF_STATE_DIM, EKF_STATE_DIM, PX, P_ARSH);
#endif
}

void EKF_AHRSGetQ(float* Q_ARSH)
{
	Q_ARSH[0] = X_ARSH[0];
	Q_ARSH[1] = X_ARSH[1];
	Q_ARSH[2] = X_ARSH[2];
	Q_ARSH[3] = X_ARSH[3];
}

void EKF_AHRSGetAngle(float* rpy)
{
	float q0q0 = X_ARSH[0] * X_ARSH[0];

	//X_ARSH-Y_ARSH-z
	CBn[0] = 2.0f * (q0q0 + X_ARSH[1] * X_ARSH[1]) - 1.0f;
	CBn[1] = 2.0f * (X_ARSH[1] * X_ARSH[2] + X_ARSH[0] * X_ARSH[3]);
	CBn[2] = 2.0f * (X_ARSH[1] * X_ARSH[3] - X_ARSH[0] * X_ARSH[2]);
	//CBn[3] = 2.0f * (X_ARSH[1] * X_ARSH[2] - X_ARSH[0] * X_ARSH[3]);
	//CBn[4] = 2.0f * (q0q0 + X_ARSH[2] * X_ARSH[2]) - 1.0f;
	CBn[5] = 2.0f * (X_ARSH[2] * X_ARSH[3] + X_ARSH[0] * X_ARSH[1]);
	//CBn[6] = 2.0f * (X_ARSH[1] * X_ARSH[3] + X_ARSH[0] * X_ARSH[2]);
	//CBn[7] = 2.0f * (X_ARSH[2] * X_ARSH[3] - X_ARSH[0] * X_ARSH[1]);
	CBn[8] = 2.0f * (q0q0 + X_ARSH[3] * X_ARSH[3]) - 1.0f;

	//roll
	rpy[0] = FastAtan2(CBn[5], CBn[8]);
	if (rpy[0] == EKF_PI)
		rpy[0] = -EKF_PI;
	//pitch
	if (CBn[2] >= 1.0f)
		rpy[1] = -EKF_HALFPI;
	else if (CBn[2] <= -1.0f)
		rpy[1] = EKF_HALFPI;
	else
		rpy[1] = FastAsin(-CBn[2]);
	//yaw
	rpy[2] = FastAtan2(CBn[1], CBn[0]);
	if (rpy[2] < 0.0f){
		rpy[2] += EKF_TWOPI;
	}
	if (rpy[2] >= EKF_TWOPI){
		rpy[2] = 0.0f;
	}

	rpy[0] = EKF_TODEG(rpy[0]);
	rpy[1] = EKF_TODEG(rpy[1]);
	rpy[2] = EKF_TODEG(rpy[2]);
}
