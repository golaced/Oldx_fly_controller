#ifndef _alt_ukf_h
#define _alt_ukf_h

#include "include.h"


extern float ALT_POS_BMP_EKF,ALT_VEL_BMP_EKF,ALT_ACC_BMP_UKF_OLDX;
extern float ALT_POS_BMP,ALT_VEL_BMP,accz_bmp;
extern float ALT_POS_SONAR,ALT_VEL_SONAR,ALT_POS_SONAR2,ALT_POS_SONAR3,ALT_POS_BMP_UKF_OLDX,ALT_VEL_BMP_UKF_OLDX;;
extern double X_ukf_baro[3];
extern double X_kf_baro[4];
extern double X_kf_baro_bmp[3], P_kf_baro[9];
extern float X_apo_height[2],x_tst[2],acc_bmp;
void ukf_baro_task1(float T);
extern float acc_body[3],acc_body_buf[3][20];
extern float acc_est,acc_z_att_corr;
extern float acc_z_acc[3];
extern float nmda[3];
#endif
