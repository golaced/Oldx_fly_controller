/*
 * File: oldx_ekf_imu.h
 *
 * MATLAB Coder version            : 3.0
 * C/C++ source code generated on  : 25-Nov-2017 09:57:40
 */

#ifndef __OLDX_EKF_IMU_H__
#define __OLDX_EKF_IMU_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "oldx_ekf_imu_types.h"

/* Function Declarations */
extern void oldx_ekf_imu(double X[7], double P[49], double yaw_in, double T,
  double gx, double gy, double gz, double ax, double ay, double az, double hx,
  double hy, double hz, double n_q, double n_w, double n_a, double n_m, double
  Att[4]);
extern void oldx_ekf_imu_initialize(void);
extern void oldx_ekf_imu_terminate(void);

#endif

/*
 * File trailer for oldx_ekf_imu.h
 *
 * [EOF]
 */
