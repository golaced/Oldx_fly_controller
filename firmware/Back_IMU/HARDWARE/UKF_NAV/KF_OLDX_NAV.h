/*
 * File: KF_OLDX_NAV.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 03-Dec-2016 20:26:50
 */

#ifndef __KF_OLDX_NAV_H__
#define __KF_OLDX_NAV_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "KF_OLDX_NAV_types.h"

/* Function Declarations */
//extern void KF_OLDX_NAV(double X[3], double P[9], const double Z[3], double U,
//  const double A[9], const double B[3], const double H[9], double ga, double gwa,
//  double g_pos, double g_spd, double T,const double X_delay[4]);
extern void KF_OLDX_NAV(double X[3], double P[9], const double Z[3], double U,
  const double A[9], const double B[3], const double H[9], double ga, double gwa,
  double g_pos, double g_spd, double T, int X_delay,const double X_delaym[4]);
extern void KF_OLDX_NAV_initialize(void);
extern void KF_OLDX_NAV_terminate(void);

#endif

/*
 * File trailer for KF_OLDX_NAV.h
 *
 * [EOF]
 */
