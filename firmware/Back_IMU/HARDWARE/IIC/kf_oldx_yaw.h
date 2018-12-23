/*
 * File: kf_oldx_yaw.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 01-Dec-2016 10:33:45
 */

#ifndef __KF_OLDX_YAW_H__
#define __KF_OLDX_YAW_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "kf_oldx_yaw_types.h"

/* Function Declarations */
extern void kf_oldx_yaw(double X[2], double P[4], const double Z[2], double U,
  double gh, double ga, double gw, double T);
extern void kf_oldx_yaw_initialize(void);
extern void kf_oldx_yaw_terminate(void);

#endif

/*
 * File trailer for kf_oldx_yaw.h
 *
 * [EOF]
 */
