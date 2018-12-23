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

#include "INS_EKF.h"
#include "FastMath.h"
#include "Quaternion.h"

//////////////////////////////////////////////////////////////////////////
//all parameters below need to be tune

#define INS_EKF_PQ_INITIAL 0.1f //init quaternion (NED-to-body) uncertainty
#define INS_EKF_PP_INITIAL 100 //init North-East-Alt position uncertainties, m
#define INS_EKF_PV_INITIAL 10.0f //init NED velocity uncertainties, m/s
#define INS_EKF_PW_INITIAL 0.01f //init XYZ gyro bias uncertainties, rad/s
#define INS_EKF_PA_INITIAL 0.1f //init XYZ accel bias uncertainties, m/s^2

#define INS_EKF_QQ_INITIAL 0.001f //quaternion process noise
#define INS_EKF_QP_INITIAL 0 //position process noise, m
#define INS_EKF_QV_INITIAL 2.0f //velocity process noise, m/s
#define INS_EKF_QW_INITIAL 0.000001f //gyro bias process noise, rad/s
#define INS_EKF_QA_INITIAL 0.000001f //accel bias process noise, m/s^2

#define INS_EKF_RM_INITIAL 0.025f
#define INS_EKF_RP_INITIAL 2.0f
#define INS_EKF_RV_INITIAL 1.0f

#define INS_EKF_GRAVITY 9.81f //local gravity m/s^2
//local magnetic declination
#define INS_EKF_DECLINATION (47.888f / 180.0f * INS_EKF_PI) //47.888f DIP in shenzhen
//////////////////////////////////////////////////////////////////////////

void INS_EKF_New(INS_EKF_Filter* ins)
{
	float32_t *F = ins->F_f32;
	float32_t *H = ins->H_f32;
	float32_t *P = ins->P_f32;
	float32_t *Q = ins->Q_f32;
	float32_t *R = ins->R_f32;
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ins->P, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins->P_f32);
	//quaternion
	P[0] = P[17] = P[34] = P[51] = INS_EKF_PQ_INITIAL;
	//position
	P[68] = P[85] = P[102] = INS_EKF_PP_INITIAL;
	//velocity
	P[119] = P[136] = P[153] = INS_EKF_PV_INITIAL;
	//gyro bias
	P[170] = P[187] = P[204] = INS_EKF_PW_INITIAL;
	//accel bias
	P[221] = P[238] = P[255] = INS_EKF_PA_INITIAL;

	arm_mat_init_f32(&ins->PX, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins->PX_f32);
	arm_mat_init_f32(&ins->PHT, INS_EKF_STATE_DIM, INS_EKF_MEASUREMENT_DIM, ins->PHT_f32);

	arm_mat_init_f32(&ins->Q, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins->Q_f32);
	//quaternion
	Q[0] = Q[17] = Q[34] = Q[51] = INS_EKF_QQ_INITIAL;
	//position
	Q[68] = Q[85] = Q[102] = INS_EKF_QP_INITIAL;
	//velocity
	Q[119] = Q[136] = Q[153] = INS_EKF_QV_INITIAL;
	//gyro bias
	Q[170] = Q[187] = Q[204] = INS_EKF_QW_INITIAL;
	//accel bias
	Q[221] = Q[238] = Q[255] = INS_EKF_QA_INITIAL;

	arm_mat_init_f32(&ins->R, INS_EKF_MEASUREMENT_DIM, INS_EKF_MEASUREMENT_DIM, ins->R_f32);
	R[0] = R[10] = R[20] = INS_EKF_RM_INITIAL;
	R[30] = R[40] = R[50] = INS_EKF_RP_INITIAL;
	R[60] = R[70] = R[80] = INS_EKF_RV_INITIAL;
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ins->K, INS_EKF_STATE_DIM, INS_EKF_MEASUREMENT_DIM, ins->K_f32);
	arm_mat_init_f32(&ins->KH, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins->KH_f32);
	arm_mat_init_f32(&ins->KHP, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins->KHP_f32);
	arm_mat_init_f32(&ins->KY, INS_EKF_STATE_DIM, 1, ins->KY_f32);

	arm_mat_init_f32(&ins->F, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins->F_f32);
	arm_mat_identity_f32(&ins->F, 1.0f);
	F[71] = 1.0f; F[88] = 1.0f; F[105] = -1.0f;
	//
	arm_mat_init_f32(&ins->FT, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins->FT_f32);
	//
	arm_mat_init_f32(&ins->H, INS_EKF_MEASUREMENT_DIM, INS_EKF_STATE_DIM, H);
	H[52] = 1.0f; H[69] = 1.0f; H[86] = 1.0f; H[103] = 1.0f; H[120] = 1.0f; H[137] = 1.0f;

	arm_mat_init_f32(&ins->HT, INS_EKF_STATE_DIM, INS_EKF_MEASUREMENT_DIM, ins->HT_f32);
	arm_mat_init_f32(&ins->HP, INS_EKF_MEASUREMENT_DIM, INS_EKF_STATE_DIM, ins->HP_f32);
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ins->S, INS_EKF_MEASUREMENT_DIM, INS_EKF_MEASUREMENT_DIM, ins->S_f32);
	arm_mat_init_f32(&ins->SI, INS_EKF_MEASUREMENT_DIM, INS_EKF_MEASUREMENT_DIM, ins->SI_f32);

	arm_mat_init_f32(&ins->X, INS_EKF_STATE_DIM, 1, ins->X_f32);
	arm_mat_zero_f32(&ins->X);
	//////////////////////////////////////////////////////////////////////////
	arm_mat_init_f32(&ins->Y, INS_EKF_MEASUREMENT_DIM, 1, ins->Y_f32);
	//////////////////////////////////////////////////////////////////////////
	//
}

void INS_EKF_Init(INS_EKF_Filter* ins, float32_t *p, float32_t *v, float32_t *accel, float32_t *mag)
{
	float32_t *X = ins->X_f32;
	// local variables
	float32_t norma, normm, normx, normy;
	float32_t declination;

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
	normm = FastSqrtI(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	normx = FastSqrtI(R[0] * R[0] + R[3] * R[3] + R[6] * R[6]);
	normy = FastSqrtI(R[1] * R[1] + R[4] * R[4] + R[7] * R[7]);
	// normalize the rotation matrix
	// normalize x axis
	R[0] *= normx; R[3] *= normx; R[6] *= normx;
	// normalize y axis
	R[1] *= normy; R[4] *= normy; R[7] *= normy;
	// normalize z axis
	R[2] *= norma; R[5] *= norma; R[8] *= norma;
	//////////////////////////////////////////////////////////////////////////
	//estimate the declination
	declination = (accel[0] * mag[0] + accel[1] * mag[1] + accel[2] * mag[2]) * norma * normm;
	ins->declination = FastAsin(declination);
	//ins->declination = INS_EKF_DECLINATION;
	ins->gravity = INS_EKF_GRAVITY;
	
	Quaternion_FromRotationMatrix(R, X);
	X[4] = p[0]; X[5] = p[1]; X[6] = p[2];
	X[7] = v[0]; X[8] = v[1]; X[9] = v[2];
}

//////////////////////////////////////////////////////////////////////////
//additional input for calculate F and H with gyroscope, accelerometer and delta time
void INS_EFK_Update(INS_EKF_Filter* ins, float32_t *mag, float32_t *p, float32_t *v, float32_t *gyro, float32_t *accel, float32_t dt)
{
	float32_t *F = ins->F_f32;
	float32_t *H = ins->H_f32;
	float32_t *X = ins->X_f32;
	float32_t *Y = ins->Y_f32;

	//////////////////////////////////////////////////////////////////////////
	float32_t q0, q1, q2, q3; //w x y z
	//float32_t halfq0,halfq1,halfq2,halfq3;
	float32_t halfdtq0,halfdtq1,halfdtq2,halfdtq3;
	float32_t /*q0q0,*/ q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float32_t _2q0, _2q1, _2q2, _2q3;
	float32_t /*_4q0,*/ _4q1, _4q2, _4q3;
	float32_t halfgx, halfgy, halfgz;
	//float32_t halfdgx, halfdgy, halfdgz;
	float32_t dax, day, daz;
	float32_t ax, ay, az;
	float32_t dtdvx, dtdvy, dtdvz;

	float32_t halfdt = 0.5f * dt;
	//////////////////////////////////////////////////////////////////////////
	float32_t Cbn[9];//Cnb[9];
	float32_t sdeclination, cdeclination;
	//stuff
	float32_t norm;
	//////////////////////////////////////////////////////////////////////////	
	//halfdgx = 0.5f * (gyro[0] - X[10]);
	//halfdgy = 0.5f * (gyro[1] - X[11]);
	//halfdgz = 0.5f * (gyro[2] - X[12]);
	halfgx = halfdt * (gyro[0] - X[10]);
	halfgy = halfdt * (gyro[1] - X[11]);
	halfgz = halfdt * (gyro[2] - X[12]);

	//
	dax = accel[0] - X[13];
	day = accel[1] - X[14];
	daz = accel[2] - X[15];
	//
	dtdvx = dax * dt; dtdvy = day * dt; dtdvz = daz * dt;

	/////////////////////////////////////////////////////////////////////////
	q0 = X[0]; q1 = X[1]; q2 = X[2]; q3 = X[3];
	//halfq0 = 0.5f * q0; halfq1 = 0.5f * q1; halfq2 = 0.5f * q2; halfq3 = 0.5f * q3;
	halfdtq0 = halfdt * q0; halfdtq1 = halfdt * q1; halfdtq2 = halfdt * q2; halfdtq3 = halfdt * q3;
	/*q0q0 = q0 * q0;*/ q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
	q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
	q2q2 = q2 * q2; q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	_2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
	/*_4q0 = 4.0f * q0;*/ _4q1 = 4.0f * q1; _4q2 = 4.0f * q2; _4q3 = 4.0f * q3;
	//////////////////////////////////////////////////////////////////////////
	//accel b frame to reference frame
	Cbn[0] = 1.0f - 2.0f *(q2q2 + q3q3); Cbn[1] = 2.0f *(q1q2 - q0q3); Cbn[2] = 2.0f * (q1q3 + q0q2);
	Cbn[3] = 2.0f * (q1q2 + q0q3); Cbn[4] = 1.0f - 2.0f * (q1q1 + q3q3); Cbn[5] = 2.0f * (q2q3 - q0q1);
	Cbn[6] = 2.0f * (q1q3 - q0q2); Cbn[7] = 2.0f *(q2q3 + q0q1); Cbn[8] = 1.0f - 2.0f * (q1q1 + q2q2);

	ax = Cbn[0] * dax + Cbn[1] * day + Cbn[2] * daz;
	ay = Cbn[3] * dax + Cbn[4] * day + Cbn[5] * daz;
	az = Cbn[6] * dax + Cbn[7] * day + Cbn[8] * daz;
	az = az + ins->gravity;
	//////////////////////////////////////////////////////////////////////////
	//X = X + dX * dt;
	//quaternion
	X[0] = q0 - halfgx * q1 - halfgy * q2 - halfgz * q3;
	X[1] = q1 + halfgx * q0 - halfgy * q3 + halfgz * q2;
	X[2] = q2 + halfgx * q3 + halfgy * q0 - halfgz * q1;
	X[3] = q3 - halfgx * q2 + halfgy * q1 + halfgz * q0;
	//North-East-Alt position
	X[4] = X[4] + X[7] * dt;
	X[5] = X[5] + X[8] * dt;
	X[6] = X[6] - X[9] * dt;
	//NED velocity
	X[7] = X[7] + ax * dt;
	X[8] = X[8] + ay * dt;
	X[9] = X[9] + az * dt;
	//////////////////////////////////////////////////////////////////////////
	//calculate linearized state dynamics, f = d(xdot)/dx
	/*
	F[0] = 0.0f; F[1] = -halfdgx; F[2] = -halfdgy; F[3] = -halfdgz; F[4] = 0; F[5] = 0; F[6] = 0; F[7] = 0; F[8] = 0; F[9] = 0; F[10] = halfq1; F[11] = halfq2; F[12] = halfq3; F[13] = 0; F[14] = 0; F[15] = 0;
	F[16] = halfdgx; F[17] = 0; F[18] = halfdgz; F[19] = -halfdgy; F[20] = 0; F[21] = 0; F[22] = 0; F[23] = 0; F[24] = 0; F[25] = 0; F[26] = -halfq0; F[27] = halfq3; F[28] = -halfq2; F[29] = 0; F[30] = 0; F[31] = 0;
	F[32] = halfdgy; F[33] = -halfdgz; F[34] = 0; F[35] = halfdgx; F[36] = 0; F[37] = 0; F[38] = 0; F[39] = 0; F[40] = 0; F[41] = 0; F[42] = -halfq3; F[43] = -halfq0; F[44] = halfq1; F[45] = 0; F[46] = 0; F[47] = 0;
	F[48] = halfdgz; F[49] = halfdgy; F[50] = -halfdgx; F[51] = 0; F[52] = 0; F[53] = 0; F[54] = 0; F[55] = 0; F[56] = 0; F[57] = 0; F[58] = halfq2; F[59] = -halfq1; F[60] = -halfq0; F[61] = 0; F[62] = 0; F[63] = 0;
	F[64] = 0; F[65] = 0; F[66] = 0; F[67] = 0; F[68] = 0; F[69] = 0; F[70] = 0; F[71] = 1.0f; F[72] = 0; F[73] = 0; F[74] = 0; F[75] = 0; F[76] = 0; F[77] = 0; F[78] = 0; F[79] = 0;
	F[80] = 0; F[81] = 0; F[82] = 0; F[83] = 0; F[84] = 0; F[85] = 0; F[86] = 0; F[87] = 0; F[88] = 1.0f; F[89] = 0; F[90] = 0; F[91] = 0; F[92] = 0; F[93] = 0; F[94] = 0; F[95] = 0;
	F[96] = 0; F[97] = 0; F[98] = 0; F[99] = 0; F[100] = 0; F[101] = 0; F[102] = 0; F[103] = 0; F[104] = 0; F[105] = -1.0f; F[106] = 0; F[107] = 0; F[108] = 0; F[109] = 0; F[110] = 0; F[111] = 0;
	F[112] = -_2q3 * day + _2q2 * daz; F[113] = _2q2 * day + _2q3 * daz; F[114] = -_4q2 * dax + _2q1 * day + _2q0 * daz; F[115] = -_2q0 * day - _4q3 * dax + _2q1 * daz; F[116] = 0; F[117] = 0; F[118] = 0; F[119] = 0; F[120] = 0; F[121] = 0; F[122] = 0; F[123] = 0; F[124] = 0; F[125] = -Cbn[0]; F[126] = -Cbn[1]; F[127] = -Cbn[2];
	F[128] = -_2q1 * daz + _2q3 * dax; F[129] = -_4q1 * day + _2q2 * dax - _2q0 * daz; F[130] = _2q1 * dax + _2q3 * daz; F[131] = -_4q3 * day + _2q0 * dax + _2q2 * daz; F[132] = 0; F[133] = 0; F[134] = 0; F[135] = 0; F[136] = 0; F[137] = 0; F[138] = 0; F[139] = 0; F[140] = 0; F[141] = -Cbn[3]; F[142] = -Cbn[4]; F[143] = -Cbn[5];
	F[144] = -_2q2 * dax + _2q1 * day; F[145] = -_4q1 * daz + _2q3 * dax + _2q0 * day; F[146] = -_2q0 * dax + _2q3 * day - _4q2 * daz; F[147] = _2q1 * dax + _2q2 * day; F[148] = 0; F[149] = 0; F[150] = 0; F[151] = 0; F[152] = 0; F[153] = 0; F[154] = 0; F[155] = 0; F[156] = 0; F[157] = -Cbn[6]; F[158] = -Cbn[7]; F[159] = -Cbn[8];
	F[160] = 0; F[161] = 0; F[162] = 0; F[163] = 0; F[164] = 0; F[165] = 0; F[166] = 0; F[167] = 0; F[168] = 0; F[169] = 0; F[170] = 0; F[171] = 0; F[172] = 0; F[173] = 0; F[174] = 0; F[175] = 0;
	F[176] = 0; F[177] = 0; F[178] = 0; F[179] = 0; F[180] = 0; F[181] = 0; F[182] = 0; F[183] = 0; F[184] = 0; F[185] = 0; F[186] = 0; F[187] = 0; F[188] = 0; F[189] = 0; F[190] = 0; F[191] = 0;
	F[192] = 0; F[193] = 0; F[194] = 0; F[195] = 0; F[196] = 0; F[197] = 0; F[198] = 0; F[199] = 0; F[200] = 0; F[201] = 0; F[202] = 0; F[203] = 0; F[204] = 0; F[205] = 0; F[206] = 0; F[207] = 0;
	F[208] = 0; F[209] = 0; F[210] = 0; F[211] = 0; F[212] = 0; F[213] = 0; F[214] = 0; F[215] = 0; F[216] = 0; F[217] = 0; F[218] = 0; F[219] = 0; F[220] = 0; F[221] = 0; F[222] = 0; F[223] = 0;
	F[224] = 0; F[225] = 0; F[226] = 0; F[227] = 0; F[228] = 0; F[229] = 0; F[230] = 0; F[231] = 0; F[232] = 0; F[233] = 0; F[234] = 0; F[235] = 0; F[236] = 0; F[237] = 0; F[238] = 0; F[239] = 0;
	F[240] = 0; F[241] = 0; F[242] = 0; F[243] = 0; F[244] = 0; F[245] = 0; F[246] = 0; F[247] = 0; F[248] = 0; F[249] = 0; F[250] = 0; F[251] = 0; F[252] = 0; F[253] = 0; F[254] = 0; F[255] = 0;
	*/
	//////////////////////////////////////////////////////////////////////////
	//convert from linearized continuous-domain state model (F,Q(t)), into discrete-domain state transformation (FT[k] & Q[k])
	//simple take two items from taylor series or you can try another way
	//FT[k] = (I + F'* dt)' = I' + (F' * dt)' = I + F * dt;
	//F[0] = F[17] = F[34] = F[51] = F[68] = F[85] = F[102] = F[119] = F[136] = F[153] = F[170] = F[187] = F[204] = F[221] = F[238] = F[255] = 1.0f;
	F[1] = -halfgx; F[2] = -halfgy; F[3] = -halfgz;F[10] = halfdtq1; F[11] = halfdtq2; F[12] = halfdtq3;
	F[16] = halfgx; F[18] = halfgz; F[19] = -halfgy; F[26] = -halfdtq0; F[27] = halfdtq3; F[28] = -halfdtq2;
	F[32] = halfgy; F[33] = -halfgz; F[35] = halfgx; F[42] = -halfdtq3; F[43] = -halfdtq0; F[44] = halfdtq1;
	F[48] = halfgz; F[49] = halfgy; F[50] = -halfgx; F[58] = halfdtq2; F[59] = -halfdtq1; F[60] = -halfdtq0;
	F[112] = -_2q3 * dtdvy + _2q2 * dtdvz; F[113] = _2q2 * dtdvy + _2q3 * dtdvz; F[114] = -_4q2 * dtdvx + _2q1 * dtdvy + _2q0 * dtdvz; F[115] = -_2q0 * dtdvy - _4q3 * dtdvx + _2q1 * dtdvz; F[125] = -Cbn[0] * dt; F[126] = -Cbn[1] * dt; F[127] = -Cbn[2] * dt;
	F[128] = -_2q1 * dtdvz + _2q3 * dtdvx; F[129] = -_4q1 * dtdvy + _2q2 * dtdvx - _2q0 * dtdvz; F[130] = _2q1 * dtdvx + _2q3 * dtdvz; F[131] = -_4q3 * dtdvy + _2q0 * dtdvx + _2q2 * dtdvz; F[141] = -Cbn[3] * dt; F[142] = -Cbn[4] * dt; F[143] = -Cbn[5] * dt;
	F[144] = -_2q2 * dtdvx + _2q1 * dtdvy; F[145] = -_4q1 * dtdvz + _2q3 * dtdvx + _2q0 * dtdvy; F[146] = -_2q0 * dtdvx + _2q3 * dtdvy - _4q2 * dtdvz; F[147] = _2q1 * dtdvx + _2q2 * dtdvy; F[157] = -Cbn[6] * dt; F[158] = -Cbn[7] * dt; F[159] = -Cbn[8] * dt;
	//
	//Q[k] = FT[k] * inv(FT[k]) * Q(t)
	//////////////////////////////////////////////////////////////////////////
	//P = F * P * F' + Q[k]
	arm_mat_trans_f32(&ins->F, &ins->FT);
	arm_mat_mult_f32(&ins->F, &ins->P, &ins->PX);
	arm_mat_mult_f32(&ins->PX, &ins->FT, &ins->P);
	arm_mat_add_f32(&ins->P, &ins->Q, &ins->P);

	//////////////////////////////////////////////////////////////////////////
	q0 = X[0]; q1 = X[1]; q2 = X[2]; q3 = X[3];
	_2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
	/*_4q0 = 4.0f * q0;*/ _4q1 = 4.0f * q1; _4q2 = 4.0f * q2; _4q3 = 4.0f * q3;
	//////////////////////////////////////////////////////////////////////////
	//magnetic declination
	FastSinCos(ins->declination, &sdeclination, &cdeclination);
	//////////////////////////////////////////////////////////////////////////
	//Cnb[0] = 1.0f - 2.0f *(q2 * q2 + q3 * q3); Cnb[1] = 2.0f * (q1 * q2 + q3 * q0); Cnb[2] = 2.0f * (q1 * q3 - q2 * q0);
	//Cnb[3] = 2.0f * (q1 * q2 - q3 * q0); Cnb[4] = 1.0f - 2.0f *(q1 * q1 + q3 * q3); Cnb[5] = 2.0f * (q2 * q3 + q1 * q0);
	//Cnb[6] = 2.0f * (q1 * q3 + q2 * q0); Cnb[7] = 2.0f * (q2 * q3 - q1 * q0); Cnb[8] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
	//Cmn[0] = cdeclination; Cmn[1] = -sdeclination; Cmn[2] = 0;
	//Cmn[3] = sdeclination; Cmn[4] = cdeclination; Cmn[5] = 0;
	//Cmn[6] = 0; Cmn[7] = 0; Cmn[8] = 1.0f;
	//mn[3] = {1.0, 0, 0}; 3D mag ned unit calculate by Cnb * Cmn * mn; (in body)
	Y[0] = (1.0f - 2.0f *(q2 * q2 + q3 * q3)) * cdeclination + 2.0f * (q1 * q2 + q3 * q0) * sdeclination;
	Y[1] = 2.0f * (q1 * q2 - q3 * q0) * cdeclination + (1.0f - 2.0f *(q1 * q1 + q3 * q3)) * sdeclination;
	Y[2] = 2.0f * (q1 * q3 + q2 * q0) * cdeclination + 2.0f * (q2 * q3 - q1 * q0) * sdeclination;
	//postion
	Y[3] = X[4]; Y[4] = X[5]; Y[5] = X[6];
	//velocity
	Y[6] = X[7]; Y[7] = X[8]; Y[8] = X[9];

	H[0] = _2q3 * sdeclination; H[1] = _2q2 * sdeclination; H[2] = _2q1 * sdeclination - _4q2 * cdeclination; H[3] = _2q0 * sdeclination - _4q3 * cdeclination; /*H[4] = 0; H[5] = 0; H[6] = 0; H[7] = 0; H[8] = 0; H[9] = 0; H[10] = 0; H[11] = 0; H[12] = 0; H[13] = 0; H[14] = 0; H[15] = 0;*/
	H[16] = -_2q3 * cdeclination; H[17] = _2q2 * cdeclination - _4q1 * sdeclination; H[18] = _2q1 * cdeclination; H[19] = - _2q0 * cdeclination - _4q3* sdeclination; /*H[20] = 0; H[21] = 0; H[22] = 0; H[23] = 0; H[24] = 0; H[25] = 0; H[26] = 0; H[27] = 0; H[28] = 0; H[29] = 0; H[30] = 0; H[31] = 0;*/
	H[32] = _2q2 * cdeclination - _2q1 * sdeclination; H[33] = _2q3 * cdeclination - _2q0 * sdeclination; H[34] = _2q0 * cdeclination + _2q3 * sdeclination; H[35] = _2q1 * cdeclination + _2q2 * sdeclination; /*H[36] = 0; H[37] = 0; H[38] = 0; H[39] = 0; H[40] = 0; H[41] = 0; H[42] = 0; H[43] = 0; H[44] = 0; H[45] = 0; H[46] = 0; H[47] = 0;*/
	/*H[48] = 0; H[49] = 0; H[50] = 0; H[51] = 0; H[52] = 1.0f; H[53] = 0; H[54] = 0; H[55] = 0; H[56] = 0; H[57] = 0; H[58] = 0; H[59] = 0; H[60] = 0; H[61] = 0; H[62] = 0; H[63] = 0;*/
	/*H[64] = 0; H[65] = 0; H[66] = 0; H[67] = 0; H[68] = 0; H[69] = 1.0f; H[70] = 0; H[71] = 0; H[72] = 0; H[73] = 0; H[74] = 0; H[75] = 0; H[76] = 0; H[77] = 0; H[78] = 0; H[79] = 0;*/
	/*H[80] = 0; H[81] = 0; H[82] = 0; H[83] = 0; H[84] = 0; H[85] = 0; H[86] = 1.0f; H[87] = 0; H[88] = 0; H[89] = 0; H[90] = 0; H[91] = 0; H[92] = 0; H[93] = 0; H[94] = 0; H[95] = 0;*/
	/*H[96] = 0; H[97] = 0; H[98] = 0; H[99] = 0; H[100] = 0; H[101] = 0; H[102] = 0; H[103] = 1.0f; H[104] = 0; H[105] = 0; H[106] = 0; H[107] = 0; H[108] = 0; H[109] = 0; H[110] = 0; H[111] = 0;*/
	/*H[112] = 0; H[113] = 0; H[114] = 0; H[115] = 0; H[116] = 0; H[117] = 0; H[118] = 0; H[119] = 0; H[120] = 1.0f; H[121] = 0; H[122] = 0; H[123] = 0; H[124] = 0; H[125] = 0; H[126] = 0; H[127] = 0;*/
	/*H[128] = 0; H[129] = 0; H[130] = 0; H[131] = 0; H[132] = 0; H[133] = 0; H[134] = 0; H[135] = 0; H[136] = 0; H[137] = 1.0f; H[138] = 0; H[139] = 0; H[140] = 0; H[141] = 0; H[142] = 0; H[143] = 0;*/
	//////////////////////////////////////////////////////////////////////////
	//K = (P * H') / (H * P * H'+ R)
	//S = H * P * H' + R;
	arm_mat_trans_f32(&ins->H, &ins->HT);
	arm_mat_mult_f32(&ins->H, &ins->P, &ins->HP);
	arm_mat_mult_f32(&ins->HP, &ins->HT, &ins->S);
	arm_mat_add_f32(&ins->S, &ins->R, &ins->S);
	//calculate Kalman gain
	//K = P * H'/ S;
	arm_mat_inverse_f32(&ins->S, &ins->SI);
	arm_mat_mult_f32(&ins->P, &ins->HT, &ins->PHT);
	arm_mat_mult_f32(&ins->PHT, &ins->SI, &ins->K);
	//////////////////////////////////////////////////////////////////////////
	//
	norm = FastSqrtI(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	mag[0] *= norm; mag[1] *= norm; mag[2] *= norm;
	//unit vector pointing to magnetic North in body coords
	Y[0] = mag[0] - Y[0]; Y[1] = mag[1] - Y[1]; Y[2] = mag[2] - Y[2];
	//north pos, east pos, altitude
	Y[3] = p[0] - Y[3]; Y[4] = p[1] - Y[4]; Y[5] = p[2] - Y[5];
	//north vel, east vel, down velocity
	Y[6] = v[0] - Y[6]; Y[7] = v[1] - Y[7]; Y[8] = v[2] - Y[8];
	//X = X + K * y
	arm_mat_mult_f32(&ins->K, &ins->Y, &ins->KY);
	arm_mat_add_f32(&ins->X, &ins->KY, &ins->X);
	//////////////////////////////////////////////////////////////////////////
	//P = (I - K * H) * P
	//P = P - K * H * P
	arm_mat_mult_f32(&ins->K, &ins->H, &ins->KH);
	arm_mat_mult_f32(&ins->KH, &ins->P, &ins->KHP);
	arm_mat_sub_f32(&ins->P, &ins->KHP, &ins->P);
	//////////////////////////////////////////////////////////////////////////
	//normalize quaternion
	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;
}

void INS_EKF_GetAngle(INS_EKF_Filter* ins, float32_t* rpy)
{
	float32_t Cnb[9];
	float32_t *X = ins->X_f32;
	//Cnb
	Cnb[0] = 2.0f * (X[0] * X[0] + X[1] * X[1]) - 1.0f;
	Cnb[1] = 2.0f * (X[1] * X[2] + X[0] * X[3]);
	Cnb[2] = 2.0f * (X[1] * X[3] - X[0] * X[2]);
	//Cnb[3] = 2.0f * (X[1] * X[2] - X[0] * X[3]);
	//Cnb[4] = 2.0f * (X[0] * X[0] + X[2] * X[2]) - 1.0f;
	Cnb[5] = 2.0f * (X[2] * X[3] + X[0] * X[1]);
	//Cnb[6] = 2.0f * (X[1] * X[3] + X[0] * X[2]);
	//Cnb[7] = 2.0f * (X[2] * X[3] - X[0] * X[1]);
	Cnb[8] = 2.0f * (X[0] * X[0] + X[3] * X[3]) - 1.0f;

	//roll
	rpy[0] = FastAtan2(Cnb[5], Cnb[8]);
	if (rpy[0] == INS_EKF_PI)
		rpy[0] = -INS_EKF_PI;
	//pitch
	if (Cnb[2] >= 1.0f)
		rpy[1] = -INS_EKF_HALFPI;
	else if (Cnb[2] <= -1.0f)
		rpy[1] = INS_EKF_HALFPI;
	else
		rpy[1] = FastAsin(-Cnb[2]);
	//yaw
	rpy[2] = FastAtan2(Cnb[1], Cnb[0]);
	if (rpy[2] < 0.0f){
		rpy[2] += INS_EKF_TWOPI;
	}
	if (rpy[2] > INS_EKF_TWOPI){
		rpy[2] = 0.0f;
	}
	rpy[0] = INS_EKF_TODEG(rpy[0]);
	rpy[1] = INS_EKF_TODEG(rpy[1]);
	rpy[2] = INS_EKF_TODEG(rpy[2]);
}
