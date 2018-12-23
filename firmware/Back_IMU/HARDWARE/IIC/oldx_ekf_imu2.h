/*
 * File: oldx_ekf_imu2.h
 *
 * MATLAB Coder version            : 3.0
 * C/C++ source code generated on  : 28-Nov-2017 21:48:26
 */

#ifndef __OLDX_EKF_IMU2_H__
#define __OLDX_EKF_IMU2_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "oldx_ekf_imu2_types.h"

/* Function Declarations */
extern void oldx_ekf_imu2(float x_apo[12], float P_apo[144], const float zFlag[3],
  const float z[9], const float param[7], float dt, float xa_apo[12], float
  Pa_apo[144], float Rot_matrix[9], float eulerAngles[3]);
extern void oldx_ekf_imu2_initialize(void);
extern void oldx_ekf_imu2_terminate(void);

#endif

/*
 * File trailer for oldx_ekf_imu2.h
 *
 * [EOF]
 */
