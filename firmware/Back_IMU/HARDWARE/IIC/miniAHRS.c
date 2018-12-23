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
#include "miniAHRS.h"
#include "miniMatrix.h"
//////////////////////////////////////////////////////////////////////////
//
//all parameters below need to be tune
#define EKF_PQ_INITIAL 0.001f
#define EKF_PWB_INITIAL 0.001f

#define EKF_QQ_INITIAL 0.05f
#define EKF_QWB_INITIAL 0.005f

#define EKF_RA_INITIAL 0.07346f//0.005346f
#define EKF_RM_INITIAL 0.1005346f//0.0005346f
//////////////////////////////////////////////////////////////////////////
//
#define UPDATE_P_COMPLICATED

#ifdef UPDATE_P_COMPLICATED
static float I[EKF_STATE_DIM * EKF_STATE_DIM] = {
	1.0f, 0, 0, 0, 0, 0, 0,
	0, 1.0f, 0, 0, 0, 0, 0,
	0, 0, 1.0f, 0, 0, 0, 0,
	0, 0, 0, 1.0f, 0, 0, 0,
	0, 0, 0, 0, 1.0f, 0, 0,
	0, 0, 0, 0, 0, 1.0f, 0,
	0, 0, 0, 0, 0, 0, 1.0f,
};
#endif

static float P[EKF_STATE_DIM * EKF_STATE_DIM] = {
	EKF_PQ_INITIAL, 0, 0, 0, 0, 0, 0,
	0, EKF_PQ_INITIAL, 0, 0, 0, 0, 0,
	0, 0, EKF_PQ_INITIAL, 0, 0, 0, 0,
	0, 0, 0, EKF_PQ_INITIAL, 0, 0, 0,
	0, 0, 0, 0, EKF_PWB_INITIAL, 0, 0,
	0, 0, 0, 0, 0, EKF_PWB_INITIAL, 0,
	0, 0, 0, 0, 0, 0, EKF_PWB_INITIAL,
};

static float Q[EKF_STATE_DIM * EKF_STATE_DIM] = {
	EKF_QQ_INITIAL, 0, 0, 0, 0, 0, 0,
	0, EKF_QQ_INITIAL, 0, 0, 0, 0, 0,
	0, 0, EKF_QQ_INITIAL, 0, 0, 0, 0,
	0, 0, 0, EKF_QQ_INITIAL, 0, 0, 0,
	0, 0, 0, 0, EKF_QWB_INITIAL, 0, 0,
	0, 0, 0, 0, 0, EKF_QWB_INITIAL, 0,
	0, 0, 0, 0, 0, 0, EKF_QWB_INITIAL,
};

static float R[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {
	EKF_RA_INITIAL, 0, 0, 0, 0, 0,
	0, EKF_RA_INITIAL, 0, 0, 0, 0,
	0, 0, EKF_RA_INITIAL, 0, 0, 0,
	0, 0, 0, EKF_RM_INITIAL, 0, 0,
	0, 0, 0, 0, EKF_RM_INITIAL, 0,
	0, 0, 0, 0, 0, EKF_RM_INITIAL,
};

static float F[EKF_STATE_DIM * EKF_STATE_DIM] = {
	1.0f, 0, 0, 0, 0, 0, 0,
	0, 1.0f, 0, 0, 0, 0, 0,
	0, 0, 1.0f, 0, 0, 0, 0,
	0, 0, 0, 1.0f, 0, 0, 0,
	0, 0, 0, 0, 1.0f, 0, 0,
	0, 0, 0, 0, 0, 1.0f, 0,
	0, 0, 0, 0, 0, 0, 1.0f,
};

static float H[EKF_MEASUREMENT_DIM * EKF_STATE_DIM] = {
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
};

//state
static float X[EKF_STATE_DIM];
static float KY[EKF_STATE_DIM];
//measurement
static float Y[EKF_MEASUREMENT_DIM];
//
static float CBn[9];
//
static float PX[EKF_STATE_DIM * EKF_STATE_DIM];
static float PXX[EKF_STATE_DIM * EKF_STATE_DIM];
static float PXY[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float K[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float S[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];

float R_a=EKF_RA_INITIAL;
float R_m=EKF_RM_INITIAL;
static void set_R(void)
{
char i;	
float R1[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {
	R_a, 0, 0, 0, 0, 0,
	0, R_a, 0, 0, 0, 0,
	0, 0, R_a, 0, 0, 0,
	0, 0, 0, R_m, 0, 0,
	0, 0, 0, 0, R_m, 0,
	0, 0, 0, 0, 0, R_m,
};
for(i=0;i<EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM;i++)
  R[i]=R1[i];

}	

static void Calcultate_RotationMatrix(float *accel, float *mag, float *R)
{
	// local variables
	float norm, fmodx, fmody;
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
	norm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	fmodx = FastSqrtI(R[0] * R[0] + R[3] * R[3] + R[6] * R[6]);
	fmody = FastSqrtI(R[1] * R[1] + R[4] * R[4] + R[7] * R[7]);
	// normalize the rotation matrix
	// normalize x axis
	R[0] *= fmodx; R[3] *= fmodx; R[6] *= fmodx;
	// normalize y axis
	R[1] *= fmody; R[4] *= fmody; R[7] *= fmody;
	// normalize z axis
	R[2] *= norm; R[5] *= norm; R[8] *= norm;
}

void EKF_AHRSInit(float *accel, float *mag)
{
	//3x3 rotation matrix
	float R[9];
	
	Calcultate_RotationMatrix(accel, mag, R);
	Quaternion_FromRotationMatrix(R, X);
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
	//set_R();
	//////////////////////////////////////////////////////////////////////////
	halfdx = halfdt * (gyro[0] - X[4]);
	halfdy = halfdt * (gyro[1] - X[5]);
	halfdz = halfdt * (gyro[2] - X[6]);
	neghalfdx = -halfdx; neghalfdy = -halfdy; neghalfdz = -halfdz;
	//
	q0 = X[0]; q1 = X[1]; q2 = X[2]; q3 = X[3];

	//////////////////////////////////////////////////////////////////////////
	//Extended Kalman Filter: Prediction Step
	//state time propagation
	//Update Quaternion with the new gyroscope measurements
	X[0] = q0 - halfdx * q1 - halfdy * q2 - halfdz * q3;
	X[1] = q1 + halfdx * q0 - halfdy * q3 + halfdz * q2;
	X[2] = q2 + halfdx * q3 + halfdy * q0 - halfdz * q1;
	X[3] = q3 - halfdx * q2 + halfdy * q1 + halfdz * q0;

	//normalize quaternion
	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;

	//populate F jacobian
	halfdtq0 = halfdt * q0; halfdtq1 = halfdt * q1; halfdtq2 = halfdt * q2; halfdtq3 = halfdt * q3;
	neghalfdtq0 = -halfdtq0; neghalfdtq1 = -halfdtq1; neghalfdtq2 = -halfdtq2; neghalfdtq3 = -halfdtq3;

	/* F[0] = 1.0f; */ F[1] = neghalfdx; F[2] = neghalfdy; F[3] = neghalfdz; F[4] = halfdtq1; F[5] = halfdtq2; F[6] = halfdtq3;
	F[7] = halfdx; /* F[8] = 1.0f; */ F[9] = halfdz;	F[10] = neghalfdy; F[11] = neghalfdtq0; F[12] = halfdtq3; F[13] = neghalfdtq2;
	F[14] = halfdy;	F[15] = neghalfdz;	/* F[16] = 1.0f; */ F[17] = halfdx; F[18] = neghalfdtq3; F[19] = neghalfdtq0; F[20] = halfdtq1;
	F[21] = halfdz; F[22] = halfdy; F[23] = neghalfdx; /* F[24] = 1.0f; */ F[25] = halfdtq2; F[26] = neghalfdtq1; F[27] = neghalfdtq0;

	//covariance time propagation
	//P = F*P*F' + Q;
	Matrix_Multiply(F, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PX);
	Matrix_Multiply_With_Transpose(PX, EKF_STATE_DIM, EKF_STATE_DIM, F, EKF_STATE_DIM, P);
	Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, Q, P);

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
	_2q0 = 2.0f * X[0]; _2q1 = 2.0f * X[1]; _2q2 = 2.0f * X[2]; _2q3 = 2.0f * X[3];
	//
	q0q0 = X[0] * X[0]; q0q1 = X[0] * X[1]; q0q2 = X[0] * X[2]; q0q3 = X[0] * X[3];
	q1q1 = X[1] * X[1]; q1q2 = X[1] * X[2]; q1q3 = X[1] * X[3];
	q2q2 = X[2] * X[2]; q2q3 = X[2] * X[3];
	q3q3 = X[3] * X[3];

	_2mx = 2.0f * mag[0]; _2my = 2.0f * mag[1]; _2mz = 2.0f * mag[2];

	hx = _2mx * (0.5f - q2q2 - q3q3) + _2my * (q1q2 - q0q3) + _2mz *(q1q3 + q0q2);
	hy = _2mx * (q1q2 + q0q3) + _2my * (0.5f - q1q1 - q3q3) + _2mz * (q2q3 - q0q1);
	hz = _2mx * (q1q3 - q0q2) + _2my * (q2q3 + q0q1) + _2mz *(0.5f - q1q1 - q2q2);
	bx = FastSqrt(hx * hx + hy * hy);
	bz = hz;
	//
	Y[0] = -2.0f * (q1q3 - q0q2);
	Y[1] = -2.0f * (q2q3 + q0q1);
	Y[2] = 1.0f - 2.0f * (q0q0 + q3q3);
	Y[3] = bx * (1.0f - 2.0f * (q2q2 + q3q3)) + bz * ( 2.0f * (q1q3 - q0q2));
	Y[4] = bx * (2.0f * (q1q2 - q0q3)) + bz * (2.0f * (q2q3 + q0q1));
	Y[5] = bx * (2.0f * (q1q3 + q0q2)) + bz * (1.0f - 2.0f * (q1q1 + q2q2));
	
	Y[0] = accel[0] - Y[0];
	Y[1] = accel[1] - Y[1];
	Y[2] = accel[2] - Y[2];
	Y[3] = mag[0] - Y[3];
	Y[4] = mag[1] - Y[4];
	Y[5] = mag[2] - Y[5];
	
	//populate H jacobian
	H[0] = _2q2; H[1] = -_2q3; H[2] = _2q0; H[3] = -_2q1;
	H[7] = -_2q1; H[8] = -_2q0; H[9] = -_2q3; H[10] = -_2q2;
	H[14] = -_2q0; H[15] = _2q1; H[16] = _2q2; H[17] = -_2q3;
	
	H[21] = bx * _2q0 - bz * _2q2; H[22] = bx * _2q1 + bz * _2q3; H[23] = -bx * _2q2 - bz * _2q0; H[24] = bz * _2q1 - bx * _2q3;
	H[28] = bz * _2q1 - bx * _2q3; H[29] = bx * _2q2 + bz * _2q0;	 H[30] = bx * _2q1 + bz * _2q3; H[31] = bz * _2q2 - bx * _2q0;
	H[35] = bx * _2q2 + bz * _2q0; H[36] = bx * _2q3 - bz * _2q1; H[37] = bx * _2q0 - bz * _2q2; H[38] = bx * _2q1 + bz * _2q3;
	
	//kalman gain calculation
	//K = P * H' / (R + H * P * H')
	//acceleration of gravity
	Matrix_Multiply_With_Transpose(P, EKF_STATE_DIM, EKF_STATE_DIM, H, EKF_MEASUREMENT_DIM, PXY);
	Matrix_Multiply(H, EKF_MEASUREMENT_DIM, EKF_STATE_DIM, PXY, EKF_MEASUREMENT_DIM, S);
	Maxtrix_Add(S, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, R, S);
	Matrix_Inverse(S, EKF_MEASUREMENT_DIM, SI);
	Matrix_Multiply(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, SI, EKF_MEASUREMENT_DIM, K);

	//update state vector
	//X = X + K * Y;
	Matrix_Multiply(K, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, Y, 1, KY);
	Maxtrix_Add(X, EKF_STATE_DIM, 1, KY, X);
	
	//normalize quaternion
	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;

	//covariance estimate update
	//P = (I - K * H) * P
	//P = P - K * H * P
	//or
	//P=(I - K*H)*P*(I - K*H)' + K*R*K'
#ifndef UPDATE_P_COMPLICATED
	Matrix_Multiply(K, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, H, EKF_STATE_DIM, PX);
	Matrix_Multiply(PX, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PXX);
	Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, PXX, P);
#else
	Matrix_Multiply(K, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, H, EKF_STATE_DIM, PX);
	Maxtrix_Sub(I, EKF_STATE_DIM, EKF_STATE_DIM, PX, PX);
	Matrix_Multiply(PX, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PXX);
	Matrix_Multiply_With_Transpose(PXX, EKF_STATE_DIM, EKF_STATE_DIM, PX, EKF_STATE_DIM, P);
	Matrix_Multiply(K, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, R, EKF_MEASUREMENT_DIM, PXY);
	Matrix_Multiply_With_Transpose(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, K, EKF_STATE_DIM, PX);
	Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, PX, P);
#endif
}

void EKF_AHRSGetQ(float* Q)
{
	Q[0] = X[0];
	Q[1] = X[1];
	Q[2] = X[2];
	Q[3] = X[3];
}

void EKF_AHRSGetAngle(float* rpy)
{
	float q0q0 = X[0] * X[0];

	//x-y-z
	CBn[0] = 2.0f * (q0q0 + X[1] * X[1]) - 1.0f;
	CBn[1] = 2.0f * (X[1] * X[2] + X[0] * X[3]);
	CBn[2] = 2.0f * (X[1] * X[3] - X[0] * X[2]);
	//CBn[3] = 2.0f * (X[1] * X[2] - X[0] * X[3]);
	//CBn[4] = 2.0f * (q0q0 + X[2] * X[2]) - 1.0f;
	CBn[5] = 2.0f * (X[2] * X[3] + X[0] * X[1]);
	//CBn[6] = 2.0f * (X[1] * X[3] + X[0] * X[2]);
	//CBn[7] = 2.0f * (X[2] * X[3] - X[0] * X[1]);
	CBn[8] = 2.0f * (q0q0 + X[3] * X[3]) - 1.0f;

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
