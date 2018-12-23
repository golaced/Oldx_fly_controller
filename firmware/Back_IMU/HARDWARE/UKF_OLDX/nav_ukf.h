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

#ifndef _nav_ukf_h
#define _nav_ukf_h


#include "alt_kf.h"
#include "LIS3MDL.h"
#include "usart_fc.h"
#include "my_math.h"
#define Q_FROM_AHRS 0

#define GPS_FROM_UBM 0 //UBM proctol
#define AQ_MAG_ENABLED 1
#define AQ_PRESSURE (float)baroAlt_fc/1000.

#define IMU_ACCX  imu_fushion.Acc.x/4096.*9.8
#define IMU_ACCY  imu_fushion.Acc.y/4096.*9.8
#define IMU_ACCZ  imu_fushion.Acc.z/4096.*9.8

#define IMU_RATEX  my_deathzoom(imu_fushion.Gyro_deg.x,0.0)*DEG_TO_RAD
#define IMU_RATEY  my_deathzoom(imu_fushion.Gyro_deg.y,0.0)*DEG_TO_RAD
#define IMU_RATEZ  my_deathzoom(imu_fushion.Gyro_deg.z,0.0)*DEG_TO_RAD*1.225

#define IMU_MAGX   imu_fushion.Mag_Val.x//855.0
#define IMU_MAGY   imu_fushion.Mag_Val.y//855.0
#define IMU_MAGZ   imu_fushion.Mag_Val.z//855.0

extern float acc_ukf_neo[3];
extern u32 dImuData_lastUpdate;
#define IMU_LASTUPD		dImuData_lastUpdate


#define UKF_LOG_SIZE		(17*sizeof(float))
#define UKF_LOG_BUF_SIZE	(UKF_LOG_SIZE*40)
//#define UKF_LOG_FNAME		"UKF"		// comment out to disable logging

#define SIM_S                   17		// states
#define SIM_M                   3		// max measurements
#define SIM_V                   12//16		// process noise
#define SIM_N                   3		// max observation noise

#define UKF_GYO_AVG_NUM		40

#define UKF_STATE_VELN		0
#define UKF_STATE_VELE		1
#define UKF_STATE_VELD		2
#define UKF_STATE_POSN		3
#define UKF_STATE_POSE		4
#define UKF_STATE_POSD		5
#define UKF_STATE_ACC_BIAS_X	6
#define UKF_STATE_ACC_BIAS_Y	7
#define UKF_STATE_ACC_BIAS_Z	8
#define UKF_STATE_GYO_BIAS_X	9
#define UKF_STATE_GYO_BIAS_Y	10
#define UKF_STATE_GYO_BIAS_Z	11
#define UKF_STATE_Q1		12
#define UKF_STATE_Q2		13
#define UKF_STATE_Q3		14
#define UKF_STATE_Q4		15
#define UKF_STATE_PRES_ALT	16

#define UKF_V_NOISE_ACC_BIAS_X	0
#define UKF_V_NOISE_ACC_BIAS_Y	1
#define UKF_V_NOISE_ACC_BIAS_Z	2
#define UKF_V_NOISE_GYO_BIAS_X	3
#define UKF_V_NOISE_GYO_BIAS_Y	4
#define UKF_V_NOISE_GYO_BIAS_Z	5
#define UKF_V_NOISE_RATE_X	6
#define UKF_V_NOISE_RATE_Y	7
#define UKF_V_NOISE_RATE_Z	8
#define UKF_V_NOISE_VELN	9
#define UKF_V_NOISE_VELE	10
#define UKF_V_NOISE_VELD	11

#define UKF_VELN		navUkfData.x[UKF_STATE_VELN]
#define UKF_VELE		navUkfData.x[UKF_STATE_VELE]
#define UKF_VELD		navUkfData.x[UKF_STATE_VELD]

#define UKF_VELN_F		Moving_Median(8,0,UKF_VELN)
#define UKF_VELE_F		Moving_Median(9,0,UKF_VELE)
#define UKF_VELD_F		Moving_Median(10,0,-UKF_VELD)

#define UKF_POSN		navUkfData.x[UKF_STATE_POSN]
#define UKF_POSE		navUkfData.x[UKF_STATE_POSE]
#define UKF_POSD		navUkfData.x[UKF_STATE_POSD]
#define UKF_ACC_BIAS_X		navUkfData.x[UKF_STATE_ACC_BIAS_X]
#define UKF_ACC_BIAS_Y		navUkfData.x[UKF_STATE_ACC_BIAS_Y]
#define UKF_ACC_BIAS_Z		navUkfData.x[UKF_STATE_ACC_BIAS_Z]
#define UKF_GYO_BIAS_X		navUkfData.x[UKF_STATE_GYO_BIAS_X]
#define UKF_GYO_BIAS_Y		navUkfData.x[UKF_STATE_GYO_BIAS_Y]
#define UKF_GYO_BIAS_Z		navUkfData.x[UKF_STATE_GYO_BIAS_Z]
#define UKF_Q1			navUkfData.x[UKF_STATE_Q1]
#define UKF_Q2			navUkfData.x[UKF_STATE_Q2]
#define UKF_Q3			navUkfData.x[UKF_STATE_Q3]
#define UKF_Q4			navUkfData.x[UKF_STATE_Q4]
#define UKF_PRES_ALT		navUkfData.x[UKF_STATE_PRES_ALT]

#define USE_PRES_ALT		 	// uncomment to use pressure altitude instead of GPS
#ifdef USE_PRES_ALT
#define UKF_ALTITUDE	UKF_PRES_ALT
#else
#define UKF_ALTITUDE	UKF_POSD
#endif

#define UKF_HIST		40
#define UKF_P0			101325.0f			    // standard static pressure at sea level

#define UKF_FLOW_ROT		-90.0f				    // optical flow mounting rotation in degrees
#define UKF_FOCAL_LENGTH	16.0f				    // 16mm
#define UKF_FOCAL_PX		(UKF_FOCAL_LENGTH / (4.0f * 6.0f) * 1000.0f)   // pixel size: 6um, binning 4 enabled


#define UKF_VEL_Q               +3.2545e-02     // +0.032544903471       0.000000350530 +0.000037342305
#define UKF_VEL_ALT_Q           +1.4483e-01     // +0.144827254833       0.000000347510 -0.000055111229
#define UKF_POS_Q               +7.1562e+03     // +7156.240473309331    0.000000352142 +2.727925965284749
#define UKF_POS_ALT_Q           +5.3884e+03     // +5388.369673129109    0.000000351319 -6.187843541372100
#define UKF_ACC_BIAS_Q          +1.3317e-03     // +0.001331748045       0.000000359470 +0.000000039113
#define UKF_GYO_BIAS_Q          +4.5256e-02     // +0.045255679186       0.000000349060 +0.000045999290
#define UKF_QUAT_Q              +5.4005e-04     // +0.000540045060       0.000000353882 +0.000000029711
#define UKF_PRES_ALT_Q          +6.3105e+01     // +63.104671424320      0.000000353790 +0.0166164673283
#define UKF_ACC_BIAS_V          +7.8673e-07     // +0.000000786725       0.000000345847 -0.000000000977
#define UKF_GYO_BIAS_V          +4.0297e-09     // +0.000000004030       0.000000359017 +0.000000000000
#define UKF_RATE_V              +1.7538e-05     // +0.000017538388       0.000000358096 +0.000000000397
#define UKF_VEL_V               +2.8605e-07     // +0.000000286054       0.000000351709 +0.000000000183
#define UKF_ALT_VEL_V           +6.8304e-08     // +0.000000068304       0.000000362348 -0.000000000050
#define UKF_GPS_POS_N           +8.0703e-06     // +0.000008070349       0.000000353490 +0.000000005602
#define UKF_GPS_POS_M_N         +3.0245e-05     // +0.000030245341       0.000000345021 -0.000000008396
#define UKF_GPS_ALT_N           +1.1796e-05     // +0.000011795879       0.000000356036 -0.000000010027
#define UKF_GPS_ALT_M_N         +3.8329e-05     // +0.000038328879       0.000000346581 +0.000000027268
#define UKF_GPS_VEL_N           +1.7640e-01     // +0.176404763511       0.000000355574 -0.000094105688
#define UKF_GPS_VEL_M_N         +3.0138e-02     // +0.030138272888       0.000000343584 -0.000002668997
#define UKF_GPS_VD_N            +4.6379e+00     // +4.637855992835       0.000000358079 +0.000310962082
#define UKF_GPS_VD_M_N          +1.3127e-02     // +0.013127146795       0.000000347978 -0.000001550944
#define UKF_ALT_N               +9.5913e-02     // +0.095913477777       0.000000356359 -0.000049781087
#define UKF_ACC_N               +6.3287e-05     // +0.000063286884       0.000000342761 -0.000000022717
#define UKF_DIST_N              +9.7373e-03     // +0.009737270392       0.000000356147 +0.000009059372
#define UKF_MAG_N               +5.2355e-01     // +0.523549973965       0.000000500000 +0.000000000000
//#define UKF_POS_DELAY_UBM       +2.1923e+03 
 //+2.1923e+03     // +2192.300048828125    0.000000500000 +0.000000000000125
//#define UKF_VEL_DELAY           -1.0182e+05     // -101820.000000000000  0.000000500000 +0.00000000000000000
#define UKF_VEL_DELAY_FLOW      UKF_VEL_DELAY

//#define UKF_POS_DELAY           +2.1923e+03     // +2192.300048828125    0.000000500000 +0.000000000000125
//#define UKF_VEL_DELAY           UKF_POS_DELAY     // -101820.000000000000  0.000000500000 +0.00000000000000000
//#define UKF_VEL_DELAY_FLOW      UKF_VEL_DELAY

typedef struct {
    srcdkf_t *kf;
    float v0a[3];
    float v0m[3];
    double holdLat, holdLon;
    double r1, r2;
    float posN[UKF_HIST];
    float posE[UKF_HIST];
    float posD[UKF_HIST];
    float velN[UKF_HIST];
    float velE[UKF_HIST];
    float velD[UKF_HIST];
    int navHistIndex;
    float yaw, pitch, roll;
    float yawCos, yawSin;
    float *x;			// states
    float flowSumX, flowSumY;
    int32_t flowSumQuality;
    float flowSumAlt;
    float flowVelX, flowVelY;
    float flowPosN, flowPosE;
    float flowQuality;
    float flowAlt;
    float flowRotCos, flowRotSin;
    uint32_t flowCount, flowAltCount;
    int logPointer;
    volatile uint8_t flowLock;
    uint8_t flowInit;
    uint8_t logHandle;
} navUkfStruct_t;

extern navUkfStruct_t navUkfData;

extern void navUkfInit(void);
extern void navUkfInertialUpdate(float T);
extern void simDoPresUpdate(float pres);
extern void simDoAccUpdate(float accX, float accY, float accZ);
extern void simDoMagUpdate(float magX, float magY, float magZ);
extern void navUkfGpsPosUpdate(uint32_t gpsMicros, double lat, double lon, float alt, float hAcc, float vAcc,float mask,float T,float posN,float posE,float posZ,u8 sel);
extern void navUkfGpsVelUpdate(uint32_t gpsMicros, float velN, float velE, float velD, float sAcc,float mask,float T,u8 sel);
extern void navUkfFlowUpdate(void);
extern void navUkfOpticalFlow(int16_t x, int16_t y, uint8_t quality, float ground);
extern void navUkfSetGlobalPositionTarget(double lat, double lon);
extern void navUkfSetHereAsPositionTarget(void);
extern void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll);
extern void navUkfZeroRate(float zRate, int axis);
extern void navUkfFinish(void);
extern void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q);
extern void navUkfResetBias(void);
extern void navUkfResetVels(void);
extern void navUkfZeroPos(void);
extern void navUkfZeroVel(void);
extern void navUkfRotateVectorByQuat(float *vr, float *v, float *q);
extern float navUkfPresToAlt(float pressure);


#define RUN_SENSOR_HIST		10				// number of timesteps to average observation sensors' data

#define ALTITUDE                 (*runData.altPos)
#define VELOCITYD                (*runData.altVel)

typedef struct {
//    OS_TID runTask;
//    OS_FlagID runFlag;

    float bestHacc;
    float accMask;
    float accHist[3][RUN_SENSOR_HIST];
    float magHist[3][RUN_SENSOR_HIST];
    float presHist[RUN_SENSOR_HIST];
    float sumAcc[3];
    float sumMag[3];
    float sumPres;
    int sensorHistIndex;
    float *altPos;
    float *altVel;
} runStruct_t;

extern runStruct_t runData;
void runTaskCode(float PosN,float PosE,float PosZ,float spdN,float spdE,float spdZ,float AQ_OUTER_TIMESTEP );
extern void runInit(void);

#define IMU_ROOM_TEMP		20.0f
#define IMU_STATIC_STD		0.05f
#define IMU_STATIC_TIMEOUT	5	// seconds

// these define where to get certain data
#define AQ_YAW			navUkfData.yaw
#define AQ_PITCH		navUkfData.pitch
#define AQ_ROLL			navUkfData.roll
#endif
