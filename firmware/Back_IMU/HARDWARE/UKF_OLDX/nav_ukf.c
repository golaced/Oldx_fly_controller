/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#include "nav_ukf.h"
#include "gps.h"
#include "ukf_task.h"
#include "ekf_ins.h"
#include "time.h"

#define TIME_UP_FL 1
#define RATE_SET 10

float UKF_VEL_DELAY   =        -1.0182e+05;
float UKF_POS_DELAY_UBM   =    -1.0182e+05;
float UKF_POS_DELAY       =    -1.0182e+05;   

float k_acc_ukf1=1;
float k_m_ukf=0.666;

u8 en_dop_gps=1;
u8 en_z_bais=1,en_bias_gz=1;

float IMU_MAG_DECL	=0.0;//
float IMU_MAG_INCL	=-65.0;//65
//pos
float UKF_GPS_POS_N_TEMP=1;
float UKF_GPS_POS_N_TEMPD=1;
float NOISE_PRESS=0.02;
//spd
#if GPS_FROM_UBM
float UKF_GPS_VEL_N_TEMP=0.000286;
float UKF_GPS_VD_N_TEMP=1;

float UKF_GPS_VEL_N_TEMP_FLOW=0.00035*4;
float UKF_GPS_VD_N_TEMP_FLOW=1; 
#else
float UKF_GPS_VEL_N_TEMP=0.0362;//0.000326;
float UKF_GPS_VD_N_TEMP=UKF_GPS_VD_N;//0.0035;

float UKF_GPS_VEL_N_TEMP_FLOW=0.00035*4;
float UKF_GPS_VD_N_TEMP_FLOW=1;
#endif
//mag
float UKF_MAG_N_TEMP=UKF_MAG_N;



navUkfStruct_t navUkfData;
u32 dImuData_lastUpdate;

float navData_presAltOffset;
float compassNormalize(float heading) {
    while (heading < 0.0f)
	heading += 360.0f;
    while (heading >= 360.0f)
	heading -= 360.0f;

    return heading;
}

// calculate the shortest distance in yaw to get from b => a
float compassDifference(float a, float b) {
    float diff = b - a;

    while (diff > 180.0f)
	diff -= 360.0f;
    while (diff <= -180.0f)
	diff += 360.0f;

    return diff;
}


static float navUkfPresToAlt(float pressure) {
    return (1.0f -  powf(pressure / UKF_P0, 0.19f)) * (1.0f / 22.558e-6f);
}

static void navUkfCalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    navUkfData.r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    navUkfData.r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

static void navUkfCalcGlobalDistance(double lat, double lon, float *posNorth, float *posEast) {
    *posNorth = (lat - navUkfData.holdLat) * navUkfData.r1;
    *posEast = (lon - navUkfData.holdLon) * navUkfData.r2;
}


static void navUkfCalcLocalDistance(float localPosN, float localPosE, float *posN, float *posE) {
    *posN = localPosN - (float)navUkfData.holdLat;
    *posE = localPosE - (float)navUkfData.holdLon;
}


void navUkfNormalizeVec3(float *vr, float *v) {
    float norm;

    norm = __sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    vr[0] = v[0] / norm;
    vr[1] = v[1] / norm;
    vr[2] = v[2] / norm;
}

void navUkfNormalizeQuat(float *qr, float *q) {
    float norm;

    norm = 1.0f / __sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

    qr[0] *= norm;
    qr[1] *= norm;
    qr[2] *= norm;
    qr[3] *= norm;
}

void crossVector3(float *vr, float *va, float *vb) {
    vr[0] = va[1] * vb[2] - vb[1] * va[2];
    vr[1] = va[2] * vb[0] - vb[2] * va[0];
    vr[2] = va[0] * vb[1] - vb[0] * va[1];
}

float dotVector3(float *va, float *vb) {
    return va[0]*vb[0] + va[1]*vb[1] + va[2]*vb[2];
}

void navUkfRotateVectorByQuat(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
}

void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q) {
    float qc[4];

    qc[0] = q[0];
    qc[1] = -q[1];
    qc[2] = -q[2];
    qc[3] = -q[3];

    navUkfRotateVectorByQuat(vr, v, qc);
}

void navUkfRotateVecByMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[0*3 + 1]*v[1] + m[0*3 + 2]*v[2];
    vr[1] = m[1*3 + 0]*v[0] + m[1*3 + 1]*v[1] + m[1*3 + 2]*v[2];
    vr[2] = m[2*3 + 0]*v[0] + m[2*3 + 1]*v[1] + m[2*3 + 2]*v[2];
}

static void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[1*3 + 0]*v[1] + m[2*3 + 0]*v[2];
    vr[1] = m[0*3 + 1]*v[0] + m[1*3 + 1]*v[1] + m[2*3 + 1]*v[2];
    vr[2] = m[0*3 + 2]*v[0] + m[1*3 + 2]*v[1] + m[2*3 + 2]*v[2];
}

static void navUkfQuatToMatrix(float *m, float *q, int normalize) {
    float sqw = q[0]*q[0];
    float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];
    float tmp1, tmp2;
    float invs;

    // get the invert square length
    if (normalize)
	invs = 1.0f / (sqx + sqy + sqz + sqw);
    else
	invs = 1.0f;

    // rotation matrix is scaled by inverse square length
    m[0*3 + 0] = ( sqx - sqy - sqz + sqw) * invs;
    m[1*3 + 1] = (-sqx + sqy - sqz + sqw) * invs;
    m[2*3 + 2] = (-sqx - sqy + sqz + sqw) * invs;

    tmp1 = q[1]*q[2];
    tmp2 = q[3]*q[0];
    m[1*3 + 0] = 2.0f * (tmp1 + tmp2) * invs;
    m[0*3 + 1] = 2.0f * (tmp1 - tmp2) * invs;

    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    m[2*3 + 0] = 2.0f * (tmp1 - tmp2) * invs;
    m[0*3 + 2] = 2.0f * (tmp1 + tmp2) * invs;

    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    m[2*3 + 1] = 2.0f * (tmp1 + tmp2) * invs;
    m[1*3 + 2] = 2.0f * (tmp1 - tmp2) * invs;
}

void navUkfMatrixExtractEuler(float *m, float *yaw, float *pitch, float *roll) {
    if (m[1*3+0] > 0.998f) { // singularity at north pole
	*pitch = atan2f(m[0*3+2], m[2*3+2]);
	*yaw = M_PI/2.0f;
	*roll = 0.0f;
    } else if (m[1*3+0] < -0.998f) { // singularity at south pole
	*pitch = atan2f(m[0*3+2] ,m[2*3+2]);
	*yaw = -M_PI/2.0f;
	*roll = 0.0f;
    }
    else {
	*pitch = atan2f(-m[2*3+0] ,m[0*3+0]);
	*yaw = asinf(m[1*3+0]);
	*roll = atan2f(-m[1*3+2], m[1*3+1]);
    }
}

void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll) {
    float q0, q1, q2, q3;

    q0 = q[1];
    q1 = q[2];
    q2 = q[3];
    q3 = q[0];

    *yaw = atan2f((2.0f * (q0 * q1 + q3 * q2)), (q3*q3 - q2*q2 - q1*q1 + q0*q0));
    *pitch = asinf(-2.0f * (q0 * q2 - q1 * q3));
    *roll = atanf((2.0f * (q1 * q2 + q0 * q3)) / (q3*q3 + q2*q2 - q1*q1 -q0*q0));
}

// result and source can be the same
static void navUkfRotateQuat(float *qOut, float *qIn, float *rate) {
    float q[4];
    float r[3];

    r[0] = rate[0] * -0.5f;
    r[1] = rate[1] * -0.5f;
    r[2] = rate[2] * -0.5f;

    q[0] = qIn[0];
    q[1] = qIn[1];
    q[2] = qIn[2];
    q[3] = qIn[3];

    // rotate
    qOut[0] =       q[0] + r[0]*q[1] + r[1]*q[2] + r[2]*q[3];
    qOut[1] = -r[0]*q[0] +      q[1] - r[2]*q[2] + r[1]*q[3];
    qOut[2] = -r[1]*q[0] + r[2]*q[1] +      q[2] - r[0]*q[3];
    qOut[3] = -r[2]*q[0] - r[1]*q[1] + r[0]*q[2] +      q[3];
}



#define IIR_ORDER_ACC_UKF 10
static double b_IIR_acc_ukf[IIR_ORDER_ACC_UKF+1] ={
0.00049945407823310042,
0.0049945407823310042 ,
0.022475433520489523  ,
0.059934489387972065  ,
0.10488535642895111  , 
0.12586242771474132  , 
0.10488535642895111  , 	
0.059934489387972065  ,	
0.022475433520489523  ,
0.0049945407823310042 ,
0.00049945407823310042	
};  //ÏµÊýb
static double a_IIR_acc_ukf[IIR_ORDER_ACC_UKF+1] ={ 
 1     ,
-1.9924014816014133,   
3.0194828633553867  ,      
-2.8185224264945168  ,  
2.0387206370625282   ,
-1.0545446210956813   ,
 0.41444626875039958  ,
-0.11571862523682841  ,
 0.022498509272218331 ,
-0.0026689123535761092	,
 0.0001487644521777628
};
static double InPut_IIR_acc_ukf[3][IIR_ORDER_ACC_UKF+1] = {0};
static double OutPut_IIR_acc_ukf[3][IIR_ORDER_ACC_UKF+1] = {0};
float acc_ukf_neo[3];
float time_update;
void navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) {
    float tmp[3], acc[3];
    float rate[3];
    float mat3x3[3*3];
    float q[4];
    int i;
time_update = Get_Cycle_T(TIME_UPDATE);			
    // assume out == in
    out = in;

    for (i = 0; i < n; i++) {
	// pos
	out[UKF_STATE_POSN*n + i] = in[UKF_STATE_POSN*n + i] + in[UKF_STATE_VELN*n + i] * dt;
	out[UKF_STATE_POSE*n + i] = in[UKF_STATE_POSE*n + i] + in[UKF_STATE_VELE*n + i] * dt;
	out[UKF_STATE_POSD*n + i] = in[UKF_STATE_POSD*n + i] - in[UKF_STATE_VELD*n + i] * dt;

	// pres alt
	out[UKF_STATE_PRES_ALT*n + i] = in[UKF_STATE_PRES_ALT*n + i] - in[UKF_STATE_VELD*n + i] * dt;

	// create rot matrix from current quat

	q[0] = in[UKF_STATE_Q1*n + i];
	q[1] = in[UKF_STATE_Q2*n + i];
	q[2] = in[UKF_STATE_Q3*n + i];
	q[3] = in[UKF_STATE_Q4*n + i];
//	#if Q_FROM_AHRS	
//  q[0]=-q_nav[1];
//	q[1]=q_nav[0];
//	q[2]=-q_nav[3];
//	q[3]=q_nav[2];
//  #endif			
	navUkfQuatToMatrix(mat3x3, q, 1);

	// acc
	tmp[0] = u[0] + in[UKF_STATE_ACC_BIAS_X*n + i];
	tmp[1] = u[1] + in[UKF_STATE_ACC_BIAS_Y*n + i];
	tmp[2] = u[2] + in[UKF_STATE_ACC_BIAS_Z*n + i]*en_z_bais;

	// rotate acc to world frame
	navUkfRotateVecByMatrix(acc, tmp, mat3x3);
	acc[2] += 9.8;
  
	
	
	static float a_br[3]={0};	
	static float acc_temp1[3]={0};
	a_br[0] =tmp[0];//(float) imu_fushion.Acc.x/4096.*9.8+ in[UKF_STATE_ACC_BIAS_X*n + i]*0;//16438.;
	a_br[1] =tmp[1];//(float) imu_fushion.Acc.y/4096.*9.8+ in[UKF_STATE_ACC_BIAS_Y*n + i]*0;//16438.;
	a_br[2] =tmp[2];//(float) imu_fushion.Acc.z/4096.*9.8+ in[UKF_STATE_ACC_BIAS_Z*n + i]*en_z_bais;//16438.;
	// acc
	acc_temp1[0] = a_br[1]*reference_vr[2]  - a_br[2]*reference_vr[1] ;
	acc_temp1[1] = a_br[2]*reference_vr[0]  - a_br[0]*reference_vr[2] ;
	acc_temp1[2] =(reference_vr[2] *a_br[2] + reference_vr[0] *a_br[0] + reference_vr[1] *a_br[1]);

	acc_neo[0]=acc_temp1[1];
	acc_neo[1]=acc_temp1[0];
	acc_neo[2]=acc_temp1[2]-1.0f*9.87;		
//	#if Q_FROM_AHRS	
//  acc[0]=acc_neo[0];
//  acc[1]=acc_neo[1];
//  acc[2]=acc_neo[2];
//  #endif
	
	static float acc_temp[3];
	
//	acc_temp[0] = IIR_I_Filter(acc[0], InPut_IIR_acc_ukf[0], OutPut_IIR_acc_ukf[0], b_IIR_acc_ukf, IIR_ORDER_ACC_UKF+1, a_IIR_acc_ukf, IIR_ORDER_ACC_UKF+1);
//	acc_temp[1] = IIR_I_Filter(acc[1], InPut_IIR_acc_ukf[1], OutPut_IIR_acc_ukf[1], b_IIR_acc_ukf, IIR_ORDER_ACC_UKF+1, a_IIR_acc_ukf, IIR_ORDER_ACC_UKF+1);
//	acc_temp[2] = IIR_I_Filter(acc[2], InPut_IIR_acc_ukf[2], OutPut_IIR_acc_ukf[2], b_IIR_acc_ukf, IIR_ORDER_ACC_UKF+1, a_IIR_acc_ukf, IIR_ORDER_ACC_UKF+1);
	#if TIME_UP_FL  
	static u8 init,init_cnt;
//	if(init_cnt++>20)init=1;
//	if(init){
	acc_temp[0] =	LIMIT(firstOrderFilter(acc[0]*k_acc_ukf1,&firstOrderFilters[ACC_UKF_LOWPASS_X],dt),-3.3,3.3);
	acc_temp[1] =	LIMIT(firstOrderFilter(acc[1]*k_acc_ukf1,&firstOrderFilters[ACC_UKF_LOWPASS_Y],dt),-3.3,3.3);
	acc_temp[2] =	LIMIT(firstOrderFilter(acc[2]*k_acc_ukf1,&firstOrderFilters[ACC_UKF_LOWPASS_Z],dt),-3.3,3.3);
//		acc_temp[0]+= ( 1 / ( 1 + 1 / ( 1.2f *3.14f *0.04 ) ) ) *(acc[0]- acc_temp[0]) ;	
//		acc_temp[1]+= ( 1 / ( 1 + 1 / ( 1.2f *3.14f *0.04 ) ) ) *(acc[1]- acc_temp[1]) ;	
//		acc_temp[2]+= ( 1 / ( 1 + 1 / ( 1.2f *3.14f *0.04 ) ) ) *(acc[2]- acc_temp[2]) ;	
//	}
//	else
//	{
//	acc_temp[0]=acc[0];
//	acc_temp[1]=acc[1];
//	acc_temp[2]=acc[2];
//	}		
	#else
	acc_temp[0]=acc[0];
	acc_temp[1]=acc[1];
	acc_temp[2]=acc[2];
	#endif
	acc_ukf_neo[Xr]=acc_temp[1];
	acc_ukf_neo[Yr]=acc_temp[0];
	acc_ukf_neo[Zr]=acc_temp[2];
// vel
	out[UKF_STATE_VELN*n + i] = in[UKF_STATE_VELN*n + i] + acc_temp[0] * dt + noise[UKF_V_NOISE_VELN*n + i];
	out[UKF_STATE_VELE*n + i] = in[UKF_STATE_VELE*n + i] + acc_temp[1] * dt + noise[UKF_V_NOISE_VELE*n + i];
	out[UKF_STATE_VELD*n + i] = in[UKF_STATE_VELD*n + i] + acc_temp[2] * dt + noise[UKF_V_NOISE_VELD*n + i];

	// acc bias
	out[UKF_STATE_ACC_BIAS_X*n + i] = in[UKF_STATE_ACC_BIAS_X*n + i] + noise[UKF_V_NOISE_ACC_BIAS_X*n + i] * dt;
	out[UKF_STATE_ACC_BIAS_Y*n + i] = in[UKF_STATE_ACC_BIAS_Y*n + i] + noise[UKF_V_NOISE_ACC_BIAS_Y*n + i] * dt;
	out[UKF_STATE_ACC_BIAS_Z*n + i] = in[UKF_STATE_ACC_BIAS_Z*n + i] + noise[UKF_V_NOISE_ACC_BIAS_Z*n + i] * dt;

	// rate = rate + bias + noise
	rate[0] = (u[3] + in[UKF_STATE_GYO_BIAS_X*n + i] + noise[UKF_V_NOISE_RATE_X*n + i]) * dt;
	rate[1] = (u[4] + in[UKF_STATE_GYO_BIAS_Y*n + i] + noise[UKF_V_NOISE_RATE_Y*n + i]) * dt;
	rate[2] = (u[5] + in[UKF_STATE_GYO_BIAS_Z*n + i]*en_bias_gz + noise[UKF_V_NOISE_RATE_Z*n + i]) * dt;

	// rotate quat
	navUkfRotateQuat(q, q, rate);
	out[UKF_STATE_Q1*n + i] = q[0];
	out[UKF_STATE_Q2*n + i] = q[1];
	out[UKF_STATE_Q3*n + i] = q[2];
	out[UKF_STATE_Q4*n + i] = q[3];
//#if Q_FROM_AHRS	
//  out[UKF_STATE_Q1*n + i]+= ( 1 / ( 1 + 1 / ( k_m_ukf *3.14f *0.04 ) ) ) *(-q_nav[1]- out[UKF_STATE_Q1*n + i]) ;//-q_nav[1];
//	out[UKF_STATE_Q2*n + i]+= ( 1 / ( 1 + 1 / ( k_m_ukf *3.14f *0.04 ) ) ) *(q_nav[0]- out[UKF_STATE_Q2*n + i]) ;//q_nav[0];
//	out[UKF_STATE_Q3*n + i]+= ( 1 / ( 1 + 1 / ( k_m_ukf *3.14f *0.04 ) ) ) *(-q_nav[3]- out[UKF_STATE_Q3*n + i]) ;//-q_nav[3];
//	out[UKF_STATE_Q4*n + i]+= ( 1 / ( 1 + 1 / ( k_m_ukf *3.14f *0.04 ) ) ) *(q_nav[2]- out[UKF_STATE_Q4*n + i]) ;//q_nav[2];
//#endif	
	// gbias
	out[UKF_STATE_GYO_BIAS_X*n + i] = in[UKF_STATE_GYO_BIAS_X*n + i] + noise[UKF_V_NOISE_GYO_BIAS_X*n + i] * dt;
	out[UKF_STATE_GYO_BIAS_Y*n + i] = in[UKF_STATE_GYO_BIAS_Y*n + i] + noise[UKF_V_NOISE_GYO_BIAS_Y*n + i] * dt;
	out[UKF_STATE_GYO_BIAS_Z*n + i] = in[UKF_STATE_GYO_BIAS_Z*n + i] + noise[UKF_V_NOISE_GYO_BIAS_Z*n + i] * dt;
    }
}

void navUkfRateUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = -x[UKF_STATE_GYO_BIAS_X+(int)u[0]] + noise[0];
}

void navUkfAccUpdate(float *u, float *x, float *noise, float *y) {
    navUkfRotateVectorByRevQuat(y, navUkfData.v0a, &x[UKF_STATE_Q1]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}


void navUkfMagUpdate(float *u, float *x, float *noise, float *y) {
  static u8 cnt;
	
	static float mag[3];	
	    // calculate mag vector based on inclination
//    if(cnt++>10){cnt=0;
//	  mag[0] = cosf(IMU_MAG_INCL * DEG_TO_RAD);
//    mag[1] = 0.0f;
//    mag[2] = -sinf(IMU_MAG_INCL * DEG_TO_RAD);

//    // rotate local mag vector to align with true north
//    navUkfData.v0m[0] = mag[0] * cosf(IMU_MAG_DECL * DEG_TO_RAD) - mag[1] * sinf(IMU_MAG_DECL  * DEG_TO_RAD);
//    navUkfData.v0m[1] = mag[1] * cosf(IMU_MAG_DECL * DEG_TO_RAD) + mag[0] * sinf(IMU_MAG_DECL  * DEG_TO_RAD);
//    navUkfData.v0m[2] = mag[2];
//    }
    navUkfRotateVectorByRevQuat(y, navUkfData.v0m, &x[UKF_STATE_Q1]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_PRES_ALT] + noise[0]; // return altitude
}

void navUkfPresGPSAltUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_PRES_ALT] + noise[0];// return pres altitude
    y[1] = x[UKF_STATE_POSD] + noise[1]; // return GPS altitude
}

void navUkfPosUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_POSN] + noise[0]; // return position
    y[1] = x[UKF_STATE_POSE] + noise[1];
    y[2] = x[UKF_STATE_POSD] + noise[2];
}

void navUkfVelUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_VELN] + noise[0]; // return velocity
    y[1] = x[UKF_STATE_VELE] + noise[1];
    y[2] = x[UKF_STATE_VELD] + noise[2];
}

void navUkfOfVelUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_VELN] + noise[0]; // velN
    y[1] = x[UKF_STATE_VELE] + noise[1]; // velE
}

void navUkfOfPosUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_POSN] + noise[0]; // posN
    y[1] = x[UKF_STATE_POSE] + noise[1]; // posE
    y[2] = x[UKF_STATE_POSD] + noise[2]; // alt
}

void navUkfFinish(void) {
    navUkfNormalizeQuat(&UKF_Q1, &UKF_Q1);
    navUkfQuatExtractEuler(&UKF_Q1, &navUkfData.yaw, &navUkfData.pitch, &navUkfData.roll);
    navUkfData.yaw = To_180_degrees(compassNormalize(navUkfData.yaw * RAD_TO_DEG));
    navUkfData.pitch *= RAD_TO_DEG;
    navUkfData.roll *= RAD_TO_DEG;
    UKF_VELN=LIMIT(UKF_VELN,-6.66,6.66);
	  UKF_VELE=LIMIT(UKF_VELE,-6.66,6.66);
	  UKF_VELD=LIMIT(UKF_VELD,-6.66,6.66);
	  UKF_ACC_BIAS_X=LIMIT(UKF_ACC_BIAS_X,-2,2);  
		UKF_ACC_BIAS_Y=LIMIT(UKF_ACC_BIAS_Y,-2,2); 
		UKF_ACC_BIAS_Y=LIMIT(UKF_ACC_BIAS_Z,-2,2); 
	  UKF_GYO_BIAS_X=LIMIT(UKF_GYO_BIAS_X,-2.1,2.1);  
		UKF_GYO_BIAS_Y=LIMIT(UKF_GYO_BIAS_Y,-2.1,2.1); 
		UKF_GYO_BIAS_Z=LIMIT(UKF_GYO_BIAS_Z,-2.1,2.1); 
    //    x' = x cos f - y sin f
    //    y' = y cos f + x sin f
    navUkfData.yawCos = cosf(navUkfData.yaw * DEG_TO_RAD);
    navUkfData.yawSin = sinf(navUkfData.yaw * DEG_TO_RAD);
}

void navUkfInertialUpdate(float T) {
    float u[6];

    u[0] = IMU_ACCX;
    u[1] = IMU_ACCY;
    u[2] = IMU_ACCZ;

    u[3] = IMU_RATEX;
    u[4] = IMU_RATEY;
    u[5] = IMU_RATEZ;

    srcdkfTimeUpdate(navUkfData.kf, u, T);//acc 

#if Q_FROM_AHRS	
UKF_Q1+= ( 1 / ( 1 + 1 / ( k_m_ukf *3.14f *0.04 ) ) ) *(-q_nav[1]- UKF_Q1) ;//-q_nav[1];
UKF_Q2+= ( 1 / ( 1 + 1 / ( k_m_ukf *3.14f *0.04 ) ) ) *(q_nav[0]- UKF_Q2) ;//q_nav[0];
UKF_Q3+= ( 1 / ( 1 + 1 / ( k_m_ukf *3.14f *0.04 ) ) ) *(-q_nav[3]- UKF_Q3) ;//-q_nav[3];
UKF_Q4+= ( 1 / ( 1 + 1 / ( k_m_ukf *3.14f *0.04 ) ) ) *(q_nav[2]- UKF_Q4) ;//q_nav[2];
#endif	
	
    // store history
    navUkfData.posN[navUkfData.navHistIndex] = UKF_POSN;
    navUkfData.posE[navUkfData.navHistIndex] = UKF_POSE;
    navUkfData.posD[navUkfData.navHistIndex] = UKF_POSD;

    navUkfData.velN[navUkfData.navHistIndex] = UKF_VELN;
    navUkfData.velE[navUkfData.navHistIndex] = UKF_VELE;
    navUkfData.velD[navUkfData.navHistIndex] = UKF_VELD;

    navUkfData.navHistIndex = (navUkfData.navHistIndex + 1) % UKF_HIST;
}

void navUkfZeroRate(float rate, int axis) {
    float noise[1];        // measurement variance
    float y[1];            // measurment(s)
    float u[1];		   // user data

    noise[0] = 0.00001f;
    y[0] = rate;
    u[0] = (float)axis;

    srcdkfMeasurementUpdate(navUkfData.kf, u, y, 1, 1, noise, navUkfRateUpdate);
}

void simDoPresUpdate(float pres) {
    float noise[2];        // measurement variance
    float y[2];            // measurment(s)

    noise[0] = UKF_ALT_N;

    noise[1] = noise[0];

    y[0] = (pres);
    y[1] = y[0];

    // if GPS altitude data has been available, only update pressure altitude
  if (navData_presAltOffset!= 0.0f)
	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 1, 1, noise, navUkfPresUpdate);
    // otherwise update pressure and GPS altitude from the single pressure reading
    else
	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 2, 2, noise, navUkfPresGPSAltUpdate);
}

void simDoAccUpdate(float accX, float accY, float accZ) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    // remove bias
    accX += UKF_ACC_BIAS_X;
    accY += UKF_ACC_BIAS_Y;
    accZ += UKF_ACC_BIAS_Z*en_z_bais;

    // normalize vector
    norm =  __sqrtf(accX*accX + accY*accY + accZ*accZ);
    y[0] = accX / norm;
    y[1] = accY / norm;
    y[2] = accZ / norm;

    noise[0] = UKF_ACC_N + ABS(9.8 - norm) * UKF_DIST_N;
    if (!(fly_ready)) {
		accX -= UKF_ACC_BIAS_X;
		accY -= UKF_ACC_BIAS_Y;
		noise[0] *= 0.001f;
    }

    noise[1] = noise[0];
    noise[2] = noise[0];

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfAccUpdate);
}
//---------------------------------------MAG---------------------------------
void simDoMagUpdate(float magX, float magY, float magZ) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    noise[0] = UKF_MAG_N_TEMP;
    if ((fly_ready))
	  noise[0] *= 0.001f;
    
		//if(fabs(AQ_PITCH)<6&&fabs(AQ_ROLL)<6)noise[0]/=2;	
    noise[1] = noise[0];
    noise[2] = noise[0];

    // normalize vector
    norm = 1.0f / __sqrtf(magX*magX + magY*magY + magZ*magZ);
    y[0] = magX * norm;
    y[1] = magY * norm;
    y[2] = magZ * norm;

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfMagUpdate);
}

void navUkfZeroPos(void) {
    float y[3];
    float noise[3];

    y[0] = 0.0f;
    y[1] = 0.0f;
    y[2] = AQ_PRESSURE;//navUkfPresToAlt(AQ_PRESSURE);

    if (fly_ready) {
	noise[0] = 1e1f;
	noise[1] = 1e1f;
	noise[2] = 1e1f;
    }
    else {
	noise[0] = 1e-7f;
	noise[1] = 1e-7f;
	noise[2] = 1.0f;
    }

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);
}

void navUkfGpsPosUpdate(uint32_t gpsMicros, double lat, double lon, float alt, float hAcc, float vAcc ,float mask,float T,float posN,float posE,float posZ,u8 sel) {
    float y[3];
    float noise[3];
    float posDelta[3];
    int histIndex;
    static u8 init;
    if (lat!=0&&!init) {init=1;
			navUkfCalcEarthRadius(lat);
    }
		
   if(init){
	   y[0]=posN;
		 y[1]=posE;
		 y[2]=alt;

	// determine how far back this GPS position update came from
	#if GPS_FROM_UBM
	histIndex = (micros() - (gpsMicros + UKF_POS_DELAY_UBM)) / (int)(1e6f * T);	 
	#else	 
	histIndex = (micros() - (gpsMicros + UKF_POS_DELAY)) / (int)(1e6f * T);
	#endif	 
	histIndex = navUkfData.navHistIndex - histIndex;
	if (histIndex < 0)
	    histIndex += UKF_HIST;
	if (histIndex < 0 || histIndex >= UKF_HIST)
	    histIndex = 0;
  
	// calculate delta from current position
	posDelta[0] = UKF_POSN - navUkfData.posN[histIndex];
	posDelta[1] = UKF_POSE - navUkfData.posE[histIndex];
	posDelta[2] = UKF_POSD - navUkfData.posD[histIndex];

	// set current position state to historic data
	UKF_POSN = navUkfData.posN[histIndex];
	UKF_POSE = navUkfData.posE[histIndex];
	UKF_POSD = navUkfData.posD[histIndex];
#if GPS_FROM_UBM
	noise[0] = mask+(UKF_GPS_POS_N + en_dop_gps*hAcc *1.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.nDOP*0.01*gpsx.pvt.nDOP*0.01) * UKF_GPS_POS_M_N) *UKF_GPS_POS_N_TEMP;
	noise[1] = mask+(UKF_GPS_POS_N + en_dop_gps*hAcc *1.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.eDOP*0.01*gpsx.pvt.eDOP*0.01) * UKF_GPS_POS_M_N) *UKF_GPS_POS_N_TEMP;
	noise[2] = mask+(UKF_GPS_ALT_N + en_dop_gps*vAcc *1.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.vDOP*0.01*gpsx.pvt.vDOP*0.01) * UKF_GPS_ALT_M_N) *UKF_GPS_POS_N_TEMPD;
#else	
	noise[0] = mask+(UKF_GPS_POS_N + en_dop_gps*hAcc *0.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.nDOP*0.01*gpsx.pvt.nDOP*0.01) * UKF_GPS_POS_M_N) *UKF_GPS_POS_N_TEMP;
	noise[1] = mask+(UKF_GPS_POS_N + en_dop_gps*hAcc *0.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.eDOP*0.01*gpsx.pvt.eDOP*0.01) * UKF_GPS_POS_M_N) *UKF_GPS_POS_N_TEMP;
	noise[2] = mask+(UKF_GPS_ALT_N + en_dop_gps*vAcc *0.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.vDOP*0.01*gpsx.pvt.vDOP*0.01) * UKF_GPS_ALT_M_N) *UKF_GPS_POS_N_TEMPD;
#endif
	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);

	// add the historic position delta back to the current state
	UKF_POSN += posDelta[0];
	UKF_POSE += posDelta[1];
	UKF_POSD += posDelta[2];
  }
    
}

void navUkfZeroVel(void) {
    float y[3];
    float noise[3];

    y[0] = 0.0f;
    y[1] = 0.0f;
    y[2] = 0.0f;

    if (fly_ready) {
	noise[0] = 5.0f;
	noise[1] = 5.0f;
	noise[2] = 2.0f;
    }
    else {
	noise[0] = 1e-7f;
	noise[1] = 1e-7f;
	noise[2] = 1e-7f;
    }

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);
}

void navUkfGpsVelUpdate(uint32_t gpsMicros, float velN, float velE, float velD, float sAcc,float mask,float T,u8 sel) {
    float y[3];
    float noise[3];
    float velDelta[3];
    int histIndex;

    y[0] = velN;
    y[1] = velE;
    y[2] = velD;

    // determine how far back this GPS velocity update came from
	if(!sel)
		histIndex = (micros() - (gpsMicros + UKF_VEL_DELAY_FLOW)) / (int)(1e6f * T);
	else
    histIndex = (micros() - (gpsMicros + UKF_VEL_DELAY)) / (int)(1e6f * T);
    histIndex = navUkfData.navHistIndex - histIndex;
		if (histIndex < 0)
		histIndex += UKF_HIST;
		if (histIndex < 0 || histIndex >= UKF_HIST)
		histIndex = 0;
   //	histIndex = 0;
    // calculate delta from current position
    velDelta[0] = UKF_VELN - navUkfData.velN[histIndex];
    velDelta[1] = UKF_VELE - navUkfData.velE[histIndex];
    velDelta[2] = UKF_VELD - navUkfData.velD[histIndex];

    // set current position state to historic data
    UKF_VELN = navUkfData.velN[histIndex];
    UKF_VELE = navUkfData.velE[histIndex];
    UKF_VELD = navUkfData.velD[histIndex];
		
		float UKF_GPS_VEL_N_TEMP1,UKF_GPS_VD_N_TEMP1;	 
		if(!sel)
		{UKF_GPS_VEL_N_TEMP1=UKF_GPS_VEL_N_TEMP_FLOW;UKF_GPS_VD_N_TEMP1=UKF_GPS_VD_N_TEMP_FLOW;}
		else
		{UKF_GPS_VEL_N_TEMP1=UKF_GPS_VEL_N_TEMP;UKF_GPS_VD_N_TEMP1=UKF_GPS_VD_N_TEMP;}
 #if GPS_FROM_UBM
		noise[0] = mask+(UKF_GPS_VEL_N + sel*en_dop_gps*sAcc *1.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.nDOP*0.01*gpsx.pvt.nDOP*0.01) * UKF_GPS_VEL_M_N)*UKF_GPS_VEL_N_TEMP1;
		noise[1] = mask+(UKF_GPS_VEL_N + sel*en_dop_gps*sAcc *1.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.eDOP*0.01*gpsx.pvt.eDOP*0.01) * UKF_GPS_VEL_M_N)*UKF_GPS_VEL_N_TEMP1;
		noise[2] = mask+(UKF_GPS_VD_N  + sel*en_dop_gps*sAcc *1.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.vDOP*0.01*gpsx.pvt.vDOP*0.01) * UKF_GPS_VD_M_N) *UKF_GPS_VD_N_TEMP1;
#else		
		noise[0] = mask+(UKF_GPS_VEL_N + sel*en_dop_gps*sAcc *0.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.nDOP*0.01*gpsx.pvt.nDOP*0.01) * UKF_GPS_VEL_M_N)*UKF_GPS_VEL_N_TEMP1;
    noise[1] = mask+(UKF_GPS_VEL_N + sel*en_dop_gps*sAcc *0.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.eDOP*0.01*gpsx.pvt.eDOP*0.01) * UKF_GPS_VEL_M_N)*UKF_GPS_VEL_N_TEMP1;
    noise[2] = mask+(UKF_GPS_VD_N  + sel*en_dop_gps*sAcc *0.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.vDOP*0.01*gpsx.pvt.vDOP*0.01) * UKF_GPS_VD_M_N) *UKF_GPS_VD_N_TEMP1;
#endif
    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);

    // add the historic position delta back to the current state
    UKF_VELN += velDelta[0];
    UKF_VELE += velDelta[1];
    UKF_VELD += velDelta[2];

}

void navUkfResetBias(void) {
    // acc bias
    UKF_ACC_BIAS_X = 0.0f;
    UKF_ACC_BIAS_Y = 0.0f;
    UKF_ACC_BIAS_Z = 0.0f;

    // gyo bias
    UKF_GYO_BIAS_X = 0.0f;
    UKF_GYO_BIAS_Y = 0.0f;
    UKF_GYO_BIAS_Z = 0.0f;
}

void navUkfResetVels(void) {
    UKF_VELN = 0.0f;
    UKF_VELE = 0.0f;
    UKF_VELD = 0.0f;
}

void navUkfInitState(void) {
    uint32_t lastUpdate;
    float acc[3], mag[3];
    float estAcc[3], estMag[3];
    float m[3*3];
    int i;

    // vel
    UKF_VELN = 0.0f;
    UKF_VELE = 0.0f;
    UKF_VELD = 0.0f;

    // pos
    UKF_POSN = 0.0f;
    UKF_POSE = 0.0f;
    UKF_POSD = AQ_PRESSURE;

    // acc bias
    UKF_ACC_BIAS_X = 0.0f;
    UKF_ACC_BIAS_Y = 0.0f;
    UKF_ACC_BIAS_Z = 0.0f;

    // gyo bias
    UKF_GYO_BIAS_X = 0.0f;
    UKF_GYO_BIAS_Y = 0.0f;
    UKF_GYO_BIAS_Z = 0.0f;

    // quat
    UKF_Q1 =  1.0f;
    UKF_Q2 =  0.0f;
    UKF_Q3 =  0.0f;
    UKF_Q4 =  0.0f;

    UKF_PRES_ALT = AQ_PRESSURE;
    i = 0;
    do {
	float rotError[3];

	mag[0] = IMU_MAGX;
	mag[1] = IMU_MAGY;
	mag[2] = IMU_MAGZ;

	acc[0] = IMU_ACCX;
	acc[1] = IMU_ACCY;
	acc[2] = IMU_ACCZ;

	navUkfNormalizeVec3(acc, acc);
	navUkfNormalizeVec3(mag, mag);

	navUkfQuatToMatrix(m, &UKF_Q1, 1);

	// rotate gravity to body frame of reference
	navUkfRotateVecByRevMatrix(estAcc, navUkfData.v0a, m);

	// rotate mags to body frame of reference
	navUkfRotateVecByRevMatrix(estMag, navUkfData.v0m, m);

	// measured error, starting with ACC vector
	rotError[0] = -(acc[2] * estAcc[1] - estAcc[2] * acc[1]) * 1.0f;
	rotError[1] = -(acc[0] * estAcc[2] - estAcc[0] * acc[2]) * 1.0f;
	rotError[2] = -(acc[1] * estAcc[0] - estAcc[1] * acc[0]) * 1.0f;

	// add in MAG vector
	if (AQ_MAG_ENABLED) {
	    rotError[0] += -(mag[2] * estMag[1] - estMag[2] * mag[1]) * 1.0f;
	    rotError[1] += -(mag[0] * estMag[2] - estMag[0] * mag[2]) * 1.0f;
	    rotError[2] += -(mag[1] * estMag[0] - estMag[1] * mag[0]) * 1.0f;
	}

        navUkfRotateQuat(&UKF_Q1, &UKF_Q1, rotError);
        navUkfNormalizeQuat(&UKF_Q1, &UKF_Q1);

	i++;
    } while (i <= UKF_GYO_AVG_NUM*5);
}

void navUkfInit(void) {
    float Q[SIM_S];		// state variance
    float V[SIM_V];		// process variance
    float mag[3];

    memset((void *)&navUkfData, 0, sizeof(navUkfData));

    navUkfData.v0a[0] = 0.0f;
    navUkfData.v0a[1] = 0.0f;
    navUkfData.v0a[2] = -1.0f;

    // calculate mag vector based on inclination
    mag[0] = cosf(IMU_MAG_INCL * DEG_TO_RAD);
    mag[1] = 0.0f;
    mag[2] = -sinf(IMU_MAG_INCL * DEG_TO_RAD);

    // rotate local mag vector to align with true north
    navUkfData.v0m[0] = mag[0] * cosf(IMU_MAG_DECL * DEG_TO_RAD) - mag[1] * sinf(IMU_MAG_DECL  * DEG_TO_RAD);
    navUkfData.v0m[1] = mag[1] * cosf(IMU_MAG_DECL * DEG_TO_RAD) + mag[0] * sinf(IMU_MAG_DECL  * DEG_TO_RAD);
    navUkfData.v0m[2] = mag[2];

    navUkfData.kf = srcdkfInit(SIM_S, SIM_M, SIM_V, SIM_N, navUkfTimeUpdate);

    navUkfData.x = srcdkfGetState(navUkfData.kf);

    Q[0] = UKF_VEL_Q;
    Q[1] = UKF_VEL_Q;
    Q[2] = UKF_VEL_ALT_Q;
    Q[3] = UKF_POS_Q;
    Q[4] = UKF_POS_Q;
    Q[5] = UKF_POS_ALT_Q;
    Q[6] = UKF_ACC_BIAS_Q;
    Q[7] = UKF_ACC_BIAS_Q;
    Q[8] = UKF_ACC_BIAS_Q;
    Q[9] = UKF_GYO_BIAS_Q;
    Q[10] = UKF_GYO_BIAS_Q;
    Q[11] = UKF_GYO_BIAS_Q;
    Q[12] = UKF_QUAT_Q;
    Q[13] = UKF_QUAT_Q;
    Q[14] = UKF_QUAT_Q;
    Q[15] = UKF_QUAT_Q;
    Q[16] = UKF_PRES_ALT_Q;

    V[UKF_V_NOISE_ACC_BIAS_X] = UKF_ACC_BIAS_V;
    V[UKF_V_NOISE_ACC_BIAS_Y] = UKF_ACC_BIAS_V;
    V[UKF_V_NOISE_ACC_BIAS_Z] = UKF_ACC_BIAS_V;
    V[UKF_V_NOISE_GYO_BIAS_X] = UKF_GYO_BIAS_V;
    V[UKF_V_NOISE_GYO_BIAS_Y] = UKF_GYO_BIAS_V;
    V[UKF_V_NOISE_GYO_BIAS_Z] = UKF_GYO_BIAS_V;
    V[UKF_V_NOISE_RATE_X] = UKF_RATE_V;
    V[UKF_V_NOISE_RATE_Y] = UKF_RATE_V;
    V[UKF_V_NOISE_RATE_Z] = UKF_RATE_V;
    V[UKF_V_NOISE_VELN] = UKF_VEL_V;
    V[UKF_V_NOISE_VELE] = UKF_VEL_V;
    V[UKF_V_NOISE_VELD] = UKF_ALT_VEL_V;

    srcdkfSetVariance(navUkfData.kf, Q, V, 0, 0);

    navUkfInitState();

    navUkfData.flowRotCos = cosf(UKF_FLOW_ROT * DEG_TO_RAD);
    navUkfData.flowRotSin = sinf(UKF_FLOW_ROT * DEG_TO_RAD);


}

// reset current sea level static pressure based on better GPS estimate
void navPressureAdjust(float altitude) {
    navData_presAltOffset = altitude - ALTITUDE;
}

//--------------------------
#include "ublox.h"
runStruct_t runData;
void runTaskCode(float PosN,float PosE,float PosZ,float spdN,float spdE,float spdZ,float AQ_OUTER_TIMESTEP ) {
static u8 init ;
static  uint32_t axis = 0;
static  uint32_t loops = 0;
static u16 loss_cnt;	
if(!init){init=1;
	runInit();
	navUkfInit();
}

	// soft start GPS accuracy
  if(gpsx.pvt.PVT_numsv>=4&&gpsx.pvt.PVT_fixtype>=1)
	{runData.accMask -= 1000/(4.6/AQ_OUTER_TIMESTEP);loss_cnt=0;}
	else 
	loss_cnt++;	
	
	if(loss_cnt>333)
  runData.accMask =1000;
	runData.accMask=LIMIT(runData.accMask,0,1000);
	navUkfInertialUpdate(AQ_OUTER_TIMESTEP);//200 hz 5ms

	// record history for acc & mag & pressure readings for smoothing purposes
	// acc
	runData.sumAcc[0] -= runData.accHist[0][runData.sensorHistIndex];
	runData.sumAcc[1] -= runData.accHist[1][runData.sensorHistIndex];
	runData.sumAcc[2] -= runData.accHist[2][runData.sensorHistIndex];

	runData.accHist[0][runData.sensorHistIndex] = IMU_ACCX;
	runData.accHist[1][runData.sensorHistIndex] = IMU_ACCY;
	runData.accHist[2][runData.sensorHistIndex] = IMU_ACCZ;

	runData.sumAcc[0] += runData.accHist[0][runData.sensorHistIndex];
	runData.sumAcc[1] += runData.accHist[1][runData.sensorHistIndex];
	runData.sumAcc[2] += runData.accHist[2][runData.sensorHistIndex];

	// mag
	runData.sumMag[0] -= runData.magHist[0][runData.sensorHistIndex];
	runData.sumMag[1] -= runData.magHist[1][runData.sensorHistIndex];
	runData.sumMag[2] -= runData.magHist[2][runData.sensorHistIndex];

	runData.magHist[0][runData.sensorHistIndex] = IMU_MAGX;
	runData.magHist[1][runData.sensorHistIndex] = IMU_MAGY;
	runData.magHist[2][runData.sensorHistIndex] = IMU_MAGZ;

	runData.sumMag[0] += runData.magHist[0][runData.sensorHistIndex];
	runData.sumMag[1] += runData.magHist[1][runData.sensorHistIndex];
	runData.sumMag[2] += runData.magHist[2][runData.sensorHistIndex];

	// pressure
	runData.sumPres -= runData.presHist[runData.sensorHistIndex];
	runData.presHist[runData.sensorHistIndex] = AQ_PRESSURE;
	runData.sumPres += runData.presHist[runData.sensorHistIndex];

	runData.sensorHistIndex = (runData.sensorHistIndex + 1) % RUN_SENSOR_HIST;

	if (!((loops+1) % RATE_SET)) {
		//#if !Q_FROM_AHRS
	   simDoAccUpdate(runData.sumAcc[0]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumAcc[1]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumAcc[2]*(1.0f / (float)RUN_SENSOR_HIST));
	  //#endif 
	}
	else if (!((loops+7) % RATE_SET)&&0) {
	   simDoPresUpdate(runData.sumPres*(1.0f / (float)RUN_SENSOR_HIST));
	}
//#ifndef USE_DIGITAL_IMU
	else if (!((loops+13) % RATE_SET) && AQ_MAG_ENABLED) {
		//#if !Q_FROM_AHRS
	   simDoMagUpdate(runData.sumMag[0]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumMag[1]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumMag[2]*(1.0f / (float)RUN_SENSOR_HIST));
	  //#endif
	}
//#endif
	#if GPS_FROM_UBM//--------------------ubm
	// only accept GPS updates if there is no optical flow
	else if (0&&gpsx.pvt.PVT_numsv>=4&&gpsx.pvt.PVT_fixtype>=1&&gpsx.ubm.lat!=0&&gpsx.ubm.gpsPosFlag == 1  && gpsx.ubm.hAcc < NAV_MIN_GPS_ACC ) {
	   float dt = Get_Cycle_T(GET_T_UKF_GPS);  
   		navUkfGpsPosUpdate(gpsx.ubm.lastPosUpdate, gpsx.ubm.lat, gpsx.ubm.lon, gpsx.ubm.height, gpsx.ubm.hAcc + runData.accMask, gpsx.ubm.vAcc , runData.accMask,dt,PosN,PosE,PosZ,1);
	    gpsx.ubm.gpsPosFlag=0;
	    // refine static sea level pressure based on better GPS altitude fixes
	    if (gpsx.ubm.hAcc < runData.bestHacc && gpsx.ubm.hAcc < NAV_MIN_GPS_ACC) {
                navPressureAdjust(gpsx.ubm.height);
		runData.bestHacc = gpsx.ubm.hAcc;
	    }
	}
	else if (gpsx.pvt.PVT_numsv>=4&&gpsx.pvt.PVT_fixtype>=1&&gpsx.ubm.lat!=0&&gpsx.ubm.gpsVelFlag == 1 &&  gpsx.ubm.sAcc < NAV_MIN_GPS_ACC/2) {
	   
  		navUkfGpsVelUpdate(gpsx.ubm.lastVelUpdate, gpsx.ubm.velN, gpsx.ubm.velE, gpsx.ubm.velD, gpsx.ubm.sAcc , runData.accMask,0,1);
	    gpsx.ubm.gpsVelFlag=0;
	}
	#else//----------------pvt
	// only accept GPS updates if there is no optical flow
	else if (0&&((gpsx.pvt.PVT_numsv>=4&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0&&gps_update[0]&&((gps_init&&gps_data_vaild)))||force_test)) {
		  gps_update[0]=0;
		  float dt = Get_Cycle_T(GET_T_UKF_GPS);
		
		if ( gpsx.pvt.PVT_Hacc*0.001< NAV_MIN_GPS_ACC &&0) 
	  navUkfGpsPosUpdate(gpsData_lastPosUpdate, gpsx.pvt.PVT_latitude, gpsx.pvt.PVT_longitude, gpsx.pvt.PVT_height, gpsx.pvt.PVT_Hacc+ runData.accMask,gpsx.pvt.PVT_Vacc , runData.accMask,dt,PosN,PosE,PosZ,1);
		//else			
    //navUkfZeroPos();		
	   //   
    if ( gpsx.pvt.PVT_Sacc*0.001< NAV_MIN_GPS_ACC/2 &&1) 
		navUkfGpsVelUpdate(gpsData_lastVelUpdate, gpsx.pvt.PVT_North_speed, gpsx.pvt.PVT_East_speed, -gpsx.pvt.PVT_Down_speed, gpsx.pvt.PVT_Sacc , runData.accMask,dt,1);
    //else
		//navUkfZeroVel();
		// refine static sea level pressure based on better GPS altitude fixes
	    if (gpsx.pvt.PVT_Hacc < runData.bestHacc && gpsx.pvt.PVT_Hacc*0.001 < NAV_MIN_GPS_ACC) {
                navPressureAdjust(gpsx.pvt.PVT_height);
		  runData.bestHacc =gpsx.pvt.PVT_Hacc;}
     
		UKF_VELN=LIMIT(UKF_VELN,-3,3);
		UKF_VELE=LIMIT(UKF_VELE,-3,3);	
	}
	#endif
	else if(0&&(module.pi_flow&&pi_flow.insert)&&pi_flow.sensor.update&&
		!(gpsx.pvt.PVT_numsv>=4&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0&&((gps_init&&gps_data_vaild)))) 
	{
	 float dt = Get_Cycle_T(GET_T_UKF_FLOW);	
	 float velNorth,velEast;	
	 pi_flow.sensor.update=0;	
	 Global_GPS_Sensor.NED_Vel[0]=velEast=-(flow_matlab_data[3]*cos(AQ_YAW*0.0173)-flow_matlab_data[2]*sin(AQ_YAW*0.0173));
   Global_GPS_Sensor.NED_Vel[1]=velNorth=(flow_matlab_data[3]*sin(AQ_YAW*0.0173)+flow_matlab_data[2]*cos(AQ_YAW*0.0173));
	 	
		if ((pi_flow.sensor.qual>188&&ALT_POS_SONAR3<3)||ALT_POS_SONAR3<0.3) 
		navUkfGpsVelUpdate(pi_flow.sensor.last_update, velNorth, velEast, -ALT_VEL, gpsx.pvt.PVT_Sacc ,runData.accMask,dt,0);
    else
		navUkfZeroVel();
		
	}
	#if GPS_FROM_UBM//--------------------ubm
	// observe zero position
	else if (!((loops+4) % RATE_SET) && (gpsx.ubm.hAcc*1.001 >= NAV_MIN_GPS_ACC) ) {
	    navUkfZeroPos();
	}
	// observer zero velocity
	else if (!((loops+10) % RATE_SET) && (gpsx.ubm.sAcc*1.001 >= NAV_MIN_GPS_ACC/2 ) ) {
	    navUkfZeroVel();
	}
	#else	
	// observe zero position
	else if (!((loops+4) % RATE_SET) && (gpsx.pvt.PVT_Hacc*0.001 >= NAV_MIN_GPS_ACC) ) {
	    navUkfZeroPos();
	}
	// observer zero velocity
	else if (!((loops+10) % RATE_SET) && (gpsx.pvt.PVT_Sacc*0.001 >= NAV_MIN_GPS_ACC/2 ) ) {
	    navUkfZeroVel();
	}
	#endif
	// observe that the rates are exactly 0 if not flying or moving
	else if (!(fly_ready)) {
	    float stdX, stdY, stdZ;
		#if !Q_FROM_AHRS	
	    arm_std_f32(runData.accHist[0], RUN_SENSOR_HIST, &stdX);
	    arm_std_f32(runData.accHist[1], RUN_SENSOR_HIST, &stdY);
	    arm_std_f32(runData.accHist[2], RUN_SENSOR_HIST, &stdZ);

	    if ((stdX + stdY + stdZ) < (IMU_STATIC_STD*2)) {
		if (!((axis + 0) % 3))
		    navUkfZeroRate(IMU_RATEX, 0);
		else if (!((axis + 1) % 3))
		    navUkfZeroRate(IMU_RATEY, 1);
		else
		    navUkfZeroRate(IMU_RATEZ, 2);
		axis++;
	    }
	  #endif		
	}

        navUkfFinish();
			#if UKF_IN_ONE_THREAD
	    static u8 cnt;
	    if(cnt++>0){cnt=0;
			altUkfProcess(AQ_PRESSURE);
			}
			#endif
        // determine which altitude estimate to use
      #if GPS_FROM_UBM//--------------------ubm
			  if (gpsx.ubm.hAcc*1.001 > 0.8f) {
			#else
        if (gpsx.pvt.PVT_Hacc*0.001 > 0.8f) {
			#endif		
            runData.altPos = &ALT_POS;//baro ukf
            runData.altVel = &ALT_VEL;
        }
        else {
            runData.altPos = &UKF_ALTITUDE;//gps
            runData.altVel = &UKF_VELD;
        }

	loops++;
    
}

void runInit(void) {
    float acc[3], mag[3];
    float pres;
    int i;

    memset((void *)&runData, 0, sizeof(runData));

    acc[0] = IMU_ACCX;
    acc[1] = IMU_ACCY;
    acc[2] = IMU_ACCZ;

    mag[0] = IMU_MAGX;
    mag[1] = IMU_MAGY;
    mag[2] = IMU_MAGZ;

    pres = AQ_PRESSURE;

    // initialize sensor history
    for (i = 0; i < RUN_SENSOR_HIST; i++) {
        runData.accHist[0][i] = acc[0];
        runData.accHist[1][i] = acc[1];
        runData.accHist[2][i] = acc[2];
        runData.magHist[0][i] = mag[0];
        runData.magHist[1][i] = mag[1];
        runData.magHist[2][i] = mag[2];
        runData.presHist[i] = pres;

        runData.sumAcc[0] += acc[0];
        runData.sumAcc[1] += acc[1];
        runData.sumAcc[2] += acc[2];
        runData.sumMag[0] += mag[0];
        runData.sumMag[1] += mag[1];
        runData.sumMag[2] += mag[2];
        runData.sumPres += 0;//pres;
    }

    runData.sensorHistIndex = 0;

    runData.bestHacc = 1000.0f;
    runData.accMask = 1000.0f;

    // use altUkf altitude & vertical velocity estimates to start with
    runData.altPos = &ALT_POS;
    runData.altVel = &ALT_VEL;

}
