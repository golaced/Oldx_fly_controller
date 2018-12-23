#include "math.h"
#include "ctrl.h"
#include "quar.h"

// void Quaternion_Add(float *r, float *a, float *b)
//{
//	r[0] = a[0] + b[0];
//	r[1] = a[1] + b[1];
//	r[2] = a[2] + b[2];
//	r[3] = a[3] + b[3];
//}

// void Quaternion_Sub(float *r, float *a, float *b)
//{
//	r[0] = a[0] - b[0];
//	r[1] = a[1] - b[1];
//	r[2] = a[2] - b[2];
//	r[3] = a[3] - b[3];
//}

// void Quaternion_Multiply(float *r, float *a, float *b)
//{
//	r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
//	r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
//	r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
//	r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
//}

// void Quaternion_Conjugate(float *r, float *a)
//{
//	r[0] = a[0];
//	r[1] = -a[1];
//	r[2] = -a[2];
//	r[3] = -a[3];
//}

// void Quaternion_Scalar(float *r, float *q, float scalar)
//{
//	r[0] = q[0] * scalar;
//	r[1] = q[1] * scalar;
//	r[2] = q[2] * scalar;
//	r[3] = q[3] * scalar;
//}

//void Quaternion_Normalize(float *q)
//{
//	float norm = FastSqrtI(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
//	q[0] *= norm;
//	q[1] *= norm;
//	q[2] *= norm;
//	q[3] *= norm;
//}

//void Quaternion_FromEuler(float *q, float *rpy)
//{
//	float sPhi2, cPhi2; // sin(phi/2) and cos(phi/2)
//	float sThe2, cThe2; // sin(theta/2) and cos(theta/2)
//	float sPsi2, cPsi2; // sin(psi/2) and cos(psi/2)
//	// calculate sines and cosines
//	
//	FastSinCos(0.5f * rpy[0], &sPhi2, &cPhi2);
//	FastSinCos(0.5f * rpy[1], &sThe2, &cThe2);
//	FastSinCos(0.5f * rpy[2], &sPsi2, &cPsi2);
//	
//	// compute the quaternion elements
//	q[0] = cPsi2 * cThe2 * cPhi2 + sPsi2 * sThe2 * sPhi2;
//	q[1] = cPsi2 * cThe2 * sPhi2 - sPsi2 * sThe2 * cPhi2;
//	q[2] = cPsi2 * sThe2 * cPhi2 + sPsi2 * cThe2 * sPhi2;
//	q[3] = sPsi2 * cThe2 * cPhi2 - cPsi2 * sThe2 * sPhi2;
//}
//void Quaternion_ToNumQ( float *q, float *rpy)
//{
//  float halfP = rpy[0]/57.3f;
//  float halfR = rpy[1]/57.0f;
//  float halfY = rpy[2]/57.0f;


//  float sinP = sin(halfP);
//  float cosP = cos(halfP);
//  float sinR = sin(halfR);
//  float cosR = cos(halfR);
//  float sinY = sin(halfY);
//  float cosY = cos(halfY);


//  q[0] = cosY*cosR*cosP + sinY*sinR*sinP;
//  q[1] = cosY*cosR*sinP - sinY*sinR*cosP;
//  q[2] = cosY*sinR*cosP + sinY*cosR*sinP;
//  q[3] = sinY*cosR*cosP - cosY*sinR*sinP;
//}
//void Quaternion_ToEuler(float *q, float* rpy)
//{
//	float R[3][3];
//	//Z-Y-X
//	R[0][0] = 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f;
//	R[0][1] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
//	R[0][2] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
//	R[1][0] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
//	R[1][1] = 2.0f * (q[0] * q[0] + q[2] * q[2]) - 1.0f;
//	R[1][2] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
//	R[2][0] = 2.0f * (q[1] * q[3] + q[0] * q[2]);
//	R[2][1] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
//	R[2][2] = 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f;

//	//roll
//	rpy[0] = FastAtan2(R[1][2], R[2][2]);
//	if (rpy[0] == PI)
//		rpy[0] = -PI_2;
//	//pitch
//	if (R[0][2] >= 1.0f)
//		rpy[1] = -PI_2;
//	else if (R[0][2] <= -1.0f)
//		rpy[1] = PI_2;
//	else
//		rpy[1] = FastAsin(-R[0][2]);
//	//yaw
//	rpy[2] = FastAtan2(R[0][1], R[0][0]);
//	if (rpy[2] < 0.0f){
//		rpy[2] += _2_PI;
//	}
//	if (rpy[2] > _2_PI){
//		rpy[2] = 0.0f;
//	}
//	rpy[0] = RADTODEG(rpy[0]);
//	rpy[1] = RADTODEG(rpy[1]);
//	rpy[2] = RADTODEG(rpy[2]);
//}

//void Quaternion_To_R(float *q, float R[3][3])
//{
//	//float R[3][3];
//	//Z-Y-X
//	R[0][0] = 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f;
//	R[0][1] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
//	R[0][2] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
//	R[1][0] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
//	R[1][1] = 2.0f * (q[0] * q[0] + q[2] * q[2]) - 1.0f;
//	R[1][2] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
//	R[2][0] = 2.0f * (q[1] * q[3] + q[0] * q[2]);
//	R[2][1] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
//	R[2][2] = 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f;
//}

//void Quaternion_To_R2(float *q, float R[3][3])
//{
//	//float R[3][3];
//	//Z-Y-X
//	R[0][0] = 1-2.0f * (q[2] * q[2] + q[3] * q[3]);
//	R[0][1] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
//	R[0][2] = 2.0f * (q[1] * q[3] + q[0] * q[2]);
//	R[1][0] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
//	R[1][1] = 1-2.0f * (q[1] * q[1] + q[3] * q[3]) ;
//	R[1][2] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
//	R[2][0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
//	R[2][1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
//	R[2][2] = 1-2.0f * (q[1] * q[1] + q[2] * q[2]);
//}


//void Eular_FromRotationMatrix(float R[3][3], float *rpy)
//{
////roll
//	rpy[0] = FastAtan2(R[1][2], R[2][2]);
//	if (rpy[0] == PI)
//		rpy[0] = -PI_2;
//	//pitch
//	if (R[0][2] >= 1.0f)
//		rpy[1] = -PI_2;
//	else if (R[0][2] <= -1.0f)
//		rpy[1] = PI_2;
//	else
//		rpy[1] = FastAsin(-R[0][2]);
//	//yaw
//	rpy[2] = FastAtan2(R[0][1], R[0][0]);
//	if (rpy[2] < 0.0f){
//		rpy[2] += _2_PI;
//	}
//	if (rpy[2] > _2_PI){
//		rpy[2] = 0.0f;
//	}
//	rpy[0] = RADTODEG(rpy[0]);
//	rpy[1] = RADTODEG(rpy[1]);
//	rpy[2] = RADTODEG(rpy[2]);
//}

//void Quaternion_FromRotationMatrix(float *R, float *Q)
//{
//#if 1
//	// calculate the trace of the matrix
//	float trace = R[0] + R[4] + R[8];
//	float s;
//	if(trace > 0){
//		s = 0.5f * FastSqrt(trace + 1.0f);
//		Q[0] = 0.25f / s;
//		Q[1] = (R[7] - R[5]) * s;
//		Q[2] = (R[2] - R[6]) * s;
//		Q[3] = (R[3] - R[1]) * s;
//	}
//	else{
//		if(R[0] > R[4] && R[0] > R[8] ){
//			s = 0.5f * FastSqrtI(1.0f + R[0] - R[4] - R[8]);
//			Q[0] = (R[7] - R[5]) * s;
//			Q[1] = 0.25f / s;
//			Q[2] = (R[1] + R[3]) * s;
//			Q[3] = (R[2] + R[6]) * s;
//		}
//		else if(R[4] > R[8]) {
//			s = 0.5f * FastSqrtI(1.0f + R[4] - R[0] - R[8]);
//			Q[0] = (R[2] - R[6]) * s;
//			Q[1] = (R[1] + R[3]) * s;
//			Q[2] = 0.25f / s;
//			Q[3] = (R[5] + R[7]) * s;
//		}
//		else{
//			s = 0.5f * FastSqrtI(1.0f + R[8] - R[0] - R[4]);
//			Q[0] = (R[3] - R[1]) * s;
//			Q[1] = (R[2] + R[6]) * s;
//			Q[2] = (R[5] + R[7]) * s;
//			Q[3] = 0.25f / s;
//		}
//	}
//#else
//	// get the instantaneous orientation quaternion
//	float fq0sq; // q0^2
//	float recip4q0; // 1/4q0
//	float fmag; // quaternion magnitude
//#define SMALLQ0 0.01F // limit where rounding errors may appear
//	// get q0^2 and q0
//	fq0sq = 0.25f * (1.0f + R[0] + R[4] + R[8]);
//	Q[0] = (float)FastSqrt(fabs(fq0sq));
//	// normal case when q0 is not small meaning rotation angle not near 180 deg
//	if (Q[0] > SMALLQ0){
//		// calculate q1 to q3
//		recip4q0 = 0.25F / Q[0];
//		Q[1] = recip4q0 * (R[5] - R[7]);
//		Q[2] = recip4q0 * (R[6] - R[2]);
//		Q[3] = recip4q0 * (R[1] - R[3]);
//	}
//	else{
//		// special case of near 180 deg corresponds to nearly symmetric matrix
//		// which is not numerically well conditioned for division by small q0
//		// instead get absolute values of q1 to q3 from leading diagonal
//		Q[1] = FastSqrt(fabs(0.5f * (1.0f + R[0]) - fq0sq));
//		Q[2] = FastSqrt(fabs(0.5f * (1.0f + R[4]) - fq0sq));
//		Q[3] = FastSqrt(fabs(0.5f * (1.0f + R[8]) - fq0sq));
//		// first assume q1 is positive and ensure q2 and q3 are consistent with q1
//		if ((R[1] + R[3]) < 0.0f){
//			// q1*q2 < 0 so q2 is negative
//			Q[2] = -Q[2];
//			if ((R[5] + R[7]) > 0.0f){
//				// q1*q2 < 0 and q2*q3 > 0 so q3 is also both negative
//				Q[3] = -Q[3];
//			}
//		}
//		else if ((R[1] + R[3]) > 0.0f){
//			if ((R[5] + R[7]) < 0.0f){
//				// q1*q2 > 0 and q2*q3 < 0 so q3 is negative
//				Q[3] = -Q[3];
//			}
//		}
//		// negate the vector components if q1 should be negative
//		if ((R[5] - R[7]) < 0.0f){
//			Q[1] = -Q[1];
//			Q[2] = -Q[2];
//			Q[3] = -Q[3];
//		}
//	}
//	// finally re-normalize
//	fmag = FastSqrtI(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
//	Q[0] *= fmag;
//	Q[1] *= fmag;
//	Q[2] *= fmag;
//	Q[3] *= fmag;
//#endif
//}

//void Quaternion_RungeKutta4(float *q, float *w, float dt, int normalize)
//{
//	float half = 0.5f;
//	float two = 2.0f;
//	float qw[4], k2[4], k3[4], k4[4];
//	float tmpq[4], tmpk[4];

//	//qw = q * w * half;
//	Quaternion_Multiply(qw, q, w);
//	Quaternion_Scalar(qw, qw, half);
//	//k2 = (q + qw * dt * half) * w * half;
//	Quaternion_Scalar(tmpk, qw, dt * half);
//	Quaternion_Add(tmpk, q, tmpk);
//	Quaternion_Multiply(k2, tmpk, w);
//	Quaternion_Scalar(k2, k2, half);
//	//k3 = (q + k2 * dt * half) * w * half;
//	Quaternion_Scalar(tmpk, k2, dt * half);
//	Quaternion_Add(tmpk, q, tmpk);
//	Quaternion_Multiply(k3, tmpk, w);
//	Quaternion_Scalar(k3, k3, half);
//	//k4 = (q + k3 * dt) * w * half;
//	Quaternion_Scalar(tmpk, k3, dt);
//	Quaternion_Add(tmpk, q, tmpk);
//	Quaternion_Multiply(k4, tmpk, w);
//	Quaternion_Scalar(k4, k4, half);
//	//q += (qw + k2 * two + k3 * two + k4) * (dt / 6);
//	Quaternion_Scalar(tmpk, k2, two);
//	Quaternion_Add(tmpq, qw, tmpk);
//	Quaternion_Scalar(tmpk, k3, two);
//	Quaternion_Add(tmpq, tmpq, tmpk);
//	Quaternion_Add(tmpq, tmpq, k4);
//	Quaternion_Scalar(tmpq, tmpq, dt / 6.0f);
//	Quaternion_Add(q, q, tmpq);

//	if (normalize){
//		Quaternion_Normalize(q);
//	}
//}

//void Quaternion_From6AxisData(float* q, float *accel, float *mag)
//{
//	// local variables
//	float norma, normx, normy;
//	//float normm;
//	//3x3 rotation matrix
//	float R[9];
//	// place the un-normalized gravity and geomagnetic vectors into
//	// the rotation matrix z and x axes
//	R[2] = accel[0]; R[5] = accel[1]; R[8] = accel[2];
//	R[0] = mag[0]; R[3] = mag[1]; R[6] = mag[2];
//	// set y vector to vector product of z and x vectors
//	R[1] = R[5] * R[6] - R[8] * R[3];
//	R[4] = R[8] * R[0] - R[2] * R[6];
//	R[7] = R[2] * R[3] - R[5] * R[0];
//	// set x vector to vector product of y and z vectors
//	R[0] = R[4] * R[8] - R[7] * R[5];
//	R[3] = R[7] * R[2] - R[1] * R[8];
//	R[6] = R[1] * R[5] - R[4] * R[2];
//	// calculate the vector moduli invert
//	norma = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
//	//normm = FastSqrtI(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
//	normx = FastSqrtI(R[0] * R[0] + R[3] * R[3] + R[6] * R[6]);
//	normy = FastSqrtI(R[1] * R[1] + R[4] * R[4] + R[7] * R[7]);
//	// normalize the rotation matrix
//	// normalize x axis
//	R[0] *= normx; R[3] *= normx; R[6] *= normx;
//	// normalize y axis
//	R[1] *= normy; R[4] *= normy; R[7] *= normy;
//	// normalize z axis
//	R[2] *= norma; R[5] *= norma; R[8] *= norma;
//	
//	Quaternion_FromRotationMatrix(R, q);
//}
////-------------------------------Qua--PX4--------------------------

//	/**
//	 * set quaternion to rotation defined by euler angles
//	 */
//void euler_to_q(float angle[3],float q[4]) {
//	float roll=angle[0];
//	float pitch=angle[1];
//	float yaw=angle[2];	
//	double cosPhi_2 = cos((roll) / 2.0);
//	double sinPhi_2 = sin((roll) / 2.0);
//	double cosTheta_2 = cos((pitch) / 2.0);
//	double sinTheta_2 = sin((pitch) / 2.0);
//	double cosPsi_2 = cos((yaw) / 2.0);
//	double sinPsi_2 = sin((yaw) / 2.0);

//	/* operations executed in double to avoid loss of precision through
//	 * consecutive multiplications. Result stored as float.
//	 */
//	q[0] = (cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
//	q[1] = (sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
//	q[2] = (cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
//	q[3] = (cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
//}
//	
//	/**
//	 * set quaternion to rotation by DCM
//	 * Reference: Shoemake, Quaternions, http://www.cs.ucr.edu/~vbz/resources/quatut.pdf
//	 */

//void dcm_to_q(float q[4] ,float dcm[3][3]) {
//	float tr = dcm[0][0] + dcm[1][1] + dcm[2][2];
//	if (tr > 0.0f) {
//		float s = sqrtf(tr + 1.0f);
//		q[0] = s * 0.5f;
//		s = 0.5f / s;
//		q[1] = (dcm[2][1] - dcm[1][2]) * s;
//		q[2] = (dcm[0][2] - dcm[2][0]) * s;
//		q[3] = (dcm[1][0] - dcm[0][1]) * s;
//	} else {
//		/* Find maximum diagonal element in dcm
//		* store index in dcm_i */
//		int dcm_i = 0;
//		for (int i = 1; i < 3; i++) {
//			if (dcm[i][i] > dcm[dcm_i][dcm_i]) {
//				dcm_i = i;
//			}
//		}
//		int dcm_j = (dcm_i + 1) % 3;
//		int dcm_k = (dcm_i + 2) % 3;
//		float s = sqrtf((dcm[dcm_i][dcm_i] - dcm[dcm_j][dcm_j] -
//		dcm[dcm_k][dcm_k]) + 1.0f);
//		q[dcm_i + 1] = s * 0.5f;
//		s = 0.5f / s;
//		q[dcm_j + 1] = (dcm[dcm_i][dcm_j] + dcm[dcm_j][dcm_i]) * s;
//		q[dcm_k + 1] = (dcm[dcm_k][dcm_i] + dcm[dcm_i][dcm_k]) * s;
//		q[0] = (dcm[dcm_k][dcm_j] - dcm[dcm_j][dcm_k]) * s;
//	}
//}

//void  q_to_euler(float data[4],float angle[3])  {
//		angle[0]=atan2f(2.0f * (data[0] * data[1] + data[2] * data[3]), 1.0f - 2.0f * (data[1] * data[1] + data[2] * data[2])),
//		angle[1]=asinf(2.0f * (data[0] * data[2] - data[3] * data[1])),
//		angle[2]=atan2f(2.0f * (data[0] * data[3] + data[1] * data[2]), 1.0f - 2.0f * (data[2] * data[2] + data[3] * data[3]));
//}
/**
	 * create rotation matrix for the quaternion
	 */
void q_to_dcm(float data[4],float dcm[3][3]){
  float aSq = data[0] * data[0];
	float bSq = data[1] * data[1];
	float cSq = data[2] * data[2];
	float dSq = data[3] * data[3];
	dcm[0][0] = aSq + bSq - cSq - dSq;
	dcm[0][1] = 2.0f * (data[1] * data[2] - data[0] * data[3]);
	dcm[0][2] = 2.0f * (data[0] * data[2] + data[1] * data[3]);
	dcm[1][0] = 2.0f * (data[1] * data[2] + data[0] * data[3]);
	dcm[1][1] = aSq - bSq + cSq - dSq;
	dcm[1][2] = 2.0f * (data[2] * data[3] - data[0] * data[1]);
	dcm[2][0] = 2.0f * (data[1] * data[3] - data[0] * data[2]);
	dcm[2][1] = 2.0f * (data[0] * data[1] + data[2] * data[3]);
	dcm[2][2] = aSq - bSq - cSq + dSq;
}

void dcm_to_euler(float euler[3],float data[3][3] )  {
		euler[1] = asinf(-data[2][0]);

		if (fabsf(euler[1] - 1.57079632679489661923) < 1.0e-3f) {
			euler[0] = 0.0f;
			euler[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) + euler[0];

		} else if (fabsf(euler[1] + 1.57079632679489661923) < 1.0e-3f) {
			euler[0] = 0.0f;
			euler[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) - euler[0];

		} else {
			euler[0] = atan2f(data[2][1], data[2][2]);
			euler[2] = atan2f(data[1][0], data[0][0]);
		}

	}
//---------------------M px4-

void M33_X( float* m1, float* m2,float* m3)
{int m=3;int n=1;int p=1;
     for(int i=0;i<3;++i)                 //²æ³ËÔËËã 
    {
        for(int j=0;j<3;++j)
        {
                for(int k=0;k<3;++k)
                {
                        (*(m3+i*p+j))+=(*(m1+i*n+k))*(*(m2+k*p+j));
                 }
        }
    }
 }

 void M33_Pluse( float in1[3][3], float in2[3][3],float out[3][3])
{u8 i,j,s;
	
out[0][0]=in1[0][0]*in2[0][0]+in1[0][1]*in2[1][0]+in1[0][2]*in2[2][0];
out[0][1]=in1[0][0]*in2[0][1]+in1[0][1]*in2[1][1]+in1[0][2]*in2[2][2];
out[0][2]=in1[0][0]*in2[0][2]+in1[0][1]*in2[1][2]+in1[0][2]*in2[2][2];
	
out[1][0]=in1[1][0]*in2[0][0]+in1[1][1]*in2[1][0]+in1[1][2]*in2[2][0];
out[1][1]=in1[1][0]*in2[0][1]+in1[1][1]*in2[1][1]+in1[1][2]*in2[2][2];
out[1][2]=in1[1][0]*in2[0][2]+in1[1][1]*in2[1][2]+in1[1][2]*in2[2][2];
	
out[2][0]=in1[2][0]*in2[0][0]+in1[2][1]*in2[1][0]+in1[2][2]*in2[2][0];
out[2][1]=in1[2][0]*in2[0][1]+in1[2][1]*in2[1][1]+in1[2][2]*in2[2][2];
out[2][2]=in1[2][0]*in2[0][2]+in1[2][1]*in2[1][2]+in1[2][2]*in2[2][2];
}

void M33_Pluse_2( float in1[3][3], float in2,float out[3][3])
{u8 i,j,s;
   for(i=0;i<3;i++)
 {
  for(j=0;j<3;j++)
  {
   
    out[i][j]=in1[i][j]*in2;
 
  }
	}
}

 void M33_add( float in1[3][3], float in2[3][3],float out[3][3])
{u8 i,j,s;
   for(i=0;i<3;i++)
 {
  for(j=0;j<3;j++)
  {

    out[i][j]=in1[i][j]+in2[i][j];
 
  }
	}
}

void M33_t(float in[3][3],float out[3][3]) //Çó×ËÌ¬¾ØÕóµÄ×ªÖÃ
{
  int i,j;
  
  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
 out[i][j]=in[j][i];
}

float e_R[3];
float exp_angle[3]={0,0,0},ero_angle_px4[4];
float angle_now[3]={0,0,0};
float Q_control[2][4]={0};
float R_control_set[3][3],R_control_ero[3][3];
float R_control_now[3][3],R_control_now_t[3][3];
float yaw_dcm;
void cal_ero_outter_so3(void){
	u8 i,j;
	//set R_sp-> R_control_set

//	Rot_sp.from_euler(_v_att_sp.roll_body, _v_att_sp.pitch_body, _v_att_sp.yaw_body);
	exp_angle[1]=except_A.x;
	exp_angle[0]=except_A.y;
	if(mode_oldx.imu_use_mid_down)
	#if EN_ATT_CAL_FC
  exp_angle[2]=-Yaw_fc1;//Yaw_fc;//(ctrl_angle_offset.z + except_A.z);	
  #else	
	exp_angle[2]=yaw_dcm = -fast_atan2(2*(-ref_q_imd_down[1]*ref_q_imd_down[2] - ref_q_imd_down[0]*ref_q_imd_down[3]),
	2*(ref_q_imd_down[0]*ref_q_imd_down[0] + ref_q_imd_down[1]*ref_q_imd_down[1]) - 1) *57.3f  ;////	Yaw_mid_down;//(ctrl_angle_offset.z + except_A.z);	
	#endif
	else
	exp_angle[2]=-Yaw;//(ctrl_angle_offset.z + except_A.z);
	
	
	//exp_angle[2]=-(ctrl_angle_offset.z + except_A.z);
	float sp = sin(exp_angle[0]*0.01745);
	float cp = cos(exp_angle[0]*0.01745);
	float sr = sin(exp_angle[1]*0.01745);
	float cr = cos(exp_angle[1]*0.01745);
	float sy = sin(exp_angle[2]*0.01745);
	float cy = cos(exp_angle[2]*0.01745);

	R_control_set[0][0] = cp * cy;
	R_control_set[0][1] = (sr * sp * cy) - (cr * sy);
	R_control_set[0][2] = (cr * sp * cy) + (sr * sy);
	R_control_set[1][0] = cp * sy;
	R_control_set[1][1] = (sr * sp * sy) + (cr * cy);
	R_control_set[1][2] = (cr * sp * sy) - (sr * cy);
	R_control_set[2][0] = -sp;
	R_control_set[2][1] = sr * cp;
	R_control_set[2][2] = cr * cp;


	//euler_to_q(exp_angle,Q_control[0]);
	//q_to_dcm(Q_control[0],R_control_set);
	//now R ->R_control_now
	//euler_to_q(angle_now,Q_control[1]);
	if(mode_oldx.imu_use_mid_down)
	#if EN_ATT_CAL_FC
	q_to_dcm(ref_q,R_control_now);
  #else	
	q_to_dcm(ref_q_imd_down,R_control_now);
	#endif
	else
	q_to_dcm(q_nav,R_control_now);
	


	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	//math::Vector <3> R_z(R(0, 2), R(1, 2), R(2, 2));
	//math::Vector <3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));
	float R_z[3]={R_control_now[0][2],R_control_now[1][2],R_control_now[2][2]};
	float R_sp_z[3]={R_control_set[0][2],R_control_set[1][2],R_control_set[2][2]};
	///* axis and sin(angle) of desired rotation */
	//math::Vector <3> e_R = R.transposed() * (R_z % R_sp_z);
	float R_x_z[3];
	M33_t(R_control_now,R_control_now_t);
	R_x_z[0]=R_z[1]*R_sp_z[2]-R_z[2]*R_sp_z[1];
	R_x_z[1]=R_z[2]*R_sp_z[0]-R_z[0]*R_sp_z[2];
	R_x_z[2]=R_z[0]*R_sp_z[1]-R_z[1]*R_sp_z[0];
	
	e_R[0]=R_control_now_t[0][0]*R_x_z[0]+R_control_now_t[0][1]*R_x_z[1]+R_control_now_t[0][2]*R_x_z[2];
	e_R[1]=R_control_now_t[1][0]*R_x_z[0]+R_control_now_t[1][1]*R_x_z[1]+R_control_now_t[1][2]*R_x_z[2];
	e_R[2]=R_control_now_t[2][0]*R_x_z[0]+R_control_now_t[2][1]*R_x_z[1]+R_control_now_t[2][2]*R_x_z[2];
	/* calculate weight for yaw control */
	//float yaw_w = R_sp(2, 2) * R_sp(2, 2);
	float yaw_w =R_control_set[2][2] * R_control_set[2][2]; 
	/* calculate angle error */
	//float e_R_z_sin = e_R.length();
	float e_R_z_sin=sqrtf(e_R[0]*e_R[0]+e_R[1]*e_R[1]+e_R[2]*e_R[2]);  
	//float e_R_z_cos = R_z * R_sp_z;
	float e_R_z_cos=R_z[0]*R_sp_z[0]+R_z[1]*R_sp_z[1]+R_z[2]*R_sp_z[2]; 
	/* calculate rotation matrix after roll/pitch only rotation */
	//math::Matrix<3, 3> R_rp;
	float R_rp[3][3];
	float e_R_cp[3][3]={0};
		if (e_R_z_sin> 0.0f) {  
		/*get axis-angle representation */  
		//float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);  
		float e_R_z_axis[3] ; 
		//Vector <3> e_R_z_axis = e_R / e_R_z_sin;
		e_R_z_axis[0]=e_R[0]/e_R_z_sin;
		e_R_z_axis[1]=e_R[1]/e_R_z_sin;
		e_R_z_axis[2]=e_R[2]/e_R_z_sin;
		//e_R = e_R_z_axis * e_R_z_angle;
		e_R[0] =e_R_z_axis[0] * e_R_z_angle;  
		e_R[1] =e_R_z_axis[1] * e_R_z_angle;  
		e_R[2] =e_R_z_axis[2] * e_R_z_angle;  
		/*cross product matrix for e_R_axis */  
   
		e_R_cp[0][1] = -e_R_z_axis[2];  
		e_R_cp[0][2] = e_R_z_axis[1];  
		e_R_cp[1][0] = e_R_z_axis[2];  
		e_R_cp[1][2] = -e_R_z_axis[0];  
		e_R_cp[2][0] = -e_R_z_axis[1];  
		e_R_cp[2][1] = e_R_z_axis[0];  
		/*rotation matrix for roll/pitch only rotation */  
		//R_control_ero= R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos)
		float temp1[3][3]={0};
		M33_Pluse(e_R_cp,e_R_cp,temp1);
		float temp2[3][3]={0};
		M33_Pluse_2 (temp1,1.0f - e_R_z_cos,temp2);
		float temp3[3][3]={0};
		M33_Pluse_2 (e_R_cp,e_R_z_sin,temp3);
		float temp4[3][3]={0};
		M33_add (temp2,temp3,temp4);	
		float temp5[3][3]={0};
		float _I[3][3]={1,0,0,0,1,0,0,0,1};
		M33_add (temp4,_I,temp5);	
		M33_Pluse (R_control_now,temp5,R_rp);	
		} else {  
		/*zero roll/pitch rotation */ // R_rp = R;
		for(i=0;i<3;i++)for(j=0;j<3;j++)R_rp[i][j]=R_control_now[i][j];
		}  
		
		/* R_rp and R_sp has the same Z axis, calculate yaw error 
    math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));  
    math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));  
    e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w; */
		float  R_sp_x[3];
		R_sp_x[0]=R_control_set[0][0];
		R_sp_x[1]=R_control_set[1][0];
		R_sp_x[2]=R_control_set[2][0];
		float  R_rp_x[3];
		R_rp_x[0]=R_rp[0][0];
		R_rp_x[1]=R_rp[1][0];
		R_rp_x[2]=R_rp[2][0];

		float R_rp_x_x_R_sp_x[3];
		M33_t(R_control_now,R_control_now_t);
		R_rp_x_x_R_sp_x[0]=R_rp_x[1]*R_sp_x[2]-R_rp_x[2]*R_sp_x[1];
		R_rp_x_x_R_sp_x[1]=R_rp_x[2]*R_sp_x[0]-R_rp_x[0]*R_sp_x[2];
		R_rp_x_x_R_sp_x[2]=R_rp_x[0]*R_sp_x[1]-R_rp_x[1]*R_sp_x[0];
		e_R[2] = atan2f(R_rp_x_x_R_sp_x[0] * R_sp_z[0]+R_rp_x_x_R_sp_x[1] * R_sp_z[1]+R_rp_x_x_R_sp_x[2] * R_sp_z[2], 
		R_rp_x[0] * R_sp_x[0]+R_rp_x[1] * R_sp_x[1]+R_rp_x[2] * R_sp_x[2]) * yaw_w;  
//    if (e_R_z_cos < 0.0f) {  
//        /* for large thrust vector rotations use another rotation method:  
//         * calculate angle and axis for R -> R_sp rotation directly */  
//        math::Quaternion q;  
//        q.from_dcm(R.transposed() * R_sp);  
//        math::Vector<3> e_R_d = q.imag();  
//        e_R_d.normalize();  
//        e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0));  
//        /* use fusion of Z axis based rotation and direct rotation */  
//        float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;  
//        e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;  
//    }  
		
	  ero_angle_px4[0]=e_R[0]*57.3;//x
		ero_angle_px4[1]=-e_R[1]*57.3;//y
		ero_angle_px4[2]=e_R[2]*57.3;//z
		ero_angle_px4[3]=yaw_w;
	}		
	
	
	
_TRA traj[10];
 
void GenerateTrajectory(float  p0,float v0,float a0,float pf,float vf,float af,float Tf,
	 char defined[3],float*a,float*b,float*g,float *cost){
char accGoalDefined=defined[0];
char posGoalDefined=defined[1];
char velGoalDefined=defined[2];
 //define starting position:
 float  delta_a = af - a0;
 float  delta_v = vf - v0 - a0*Tf;
 float  delta_p = pf - p0 - v0*Tf - 0.5*a0*Tf*Tf;

 // %powers of the end time:
  float  T2 = Tf*Tf;
  float  T3 = T2*Tf;
  float  T4 = T3*Tf;
  float  T5 = T4*Tf;
  
  //%solve the trajectories, depending on what's constrained:
  if(posGoalDefined && velGoalDefined && accGoalDefined)
  {
    *a = ( 60*T2*delta_a - 360*Tf*delta_v + 720* 1*delta_p)/T5;
    *b = (-24*T3*delta_a + 168*T2*delta_v - 360*Tf*delta_p)/T5;
    *g = (  3*T4*delta_a -  24*T3*delta_v +  60*T2*delta_p)/T5;
}
  else if(posGoalDefined && velGoalDefined)
  {
    *a = (-120*Tf*delta_v + 320*   delta_p)/T5;
    *b = (  72*T2*delta_v - 200*Tf*delta_p)/T5;
    *g = ( -12*T3*delta_v +  40*T2*delta_p)/T5;
}
      else if(posGoalDefined && accGoalDefined)
			{
    *a = (-15*T2*delta_a + 90*   delta_p)/(2*T5);
    *b = ( 15*T3*delta_a - 90*Tf*delta_p)/(2*T5);
    *g = (- 3*T4*delta_a + 30*T2*delta_p)/(2*T5);
		}
      else if(velGoalDefined && accGoalDefined)
			{
    *a = 0;
    *b = ( 6*Tf*delta_a - 12*   delta_v)/T3;
    *g = (-2*T2*delta_a +  6*Tf*delta_v)/T3;
		}
      else if(posGoalDefined)
  
  {  *a =  20*delta_p/T5;
    *b = -20*delta_p/T4;
    *g =  10*delta_p/T3;
		}
      else if(velGoalDefined)
			{
    *a = 0;
    *b =-3*delta_v/T3;
    *g = 3*delta_v/T2;
		}
      else if(accGoalDefined)
			{
    *a = 0;
    *b = 0;
    *g = delta_a/Tf;
		}
  else{
 //%Nothing to do!
    *a = 0;
    *b = 0;
    *g = 0;
}

 //%Calculate the cost:
  *cost =  *g* *g + *b* *g*Tf + *b* *b*T2/3.0 + *a* *g*T2/3.0 + *a* *b*T3/4.0 + *a* *a*T4/20.0;
}

void get_trajecotry(float p0,float v0,float a0,float a,float b,float g,float t,float *pos,float *spd,float *acc,float *jerk){
*pos=p0 + v0*t + (1/2.0)*a0*t*t + (1/6.0)*g*t*t*t + (1/24.0)*b*t*t*t*t + (1/120.0)*a*t*t*t*t*t;
*jerk=g  + b*t  + (1/2.0)*a*t*t;
*acc=a0 + g*t  + (1/2.0)*b*t*t  + (1/6.0)*a*t*t*t;
*spd=v0 + a0*t + (1/2.0)*g*t*t  + (1/6.0)*b*t*t*t + (1/24.0)*a*t*t*t*t;
}


void plan_tra(_TRA *tra)
{	
 char i,j;
 float cost[3];
	for(i=0;i<3;i++)
	{
	  GenerateTrajectory(tra->ps[i],tra->vs[i],tra->as[i], tra->pe[i],tra->ve[i],tra->ae[i], tra->Time,
	  tra->defined,&tra->param[i*3+0],&tra->param[i*3+1],&tra->param[i*3+2],&cost[i]);
	}
	tra->cost_all=cost[0]+cost[1]+cost[2];
}

void get_tra(_TRA *tra,float t)
{
char i;
float acc,jerk;
for(i=0;i<3;i++)
 get_trajecotry( tra->ps[i],tra->vs[i],tra->as[i],
	tra->param[i*3+0],tra->param[i*3+1],tra->param[i*3+2]
	, t,
	&tra->pt[i], &tra->vt[i], &acc, &jerk);
}	
