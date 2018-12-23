/*
 * File: kf_oldx.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Nov-2016 14:12:33
 */

#ifndef __KF_OLDX_H__
#define __KF_OLDX_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "kf_oldx_types.h"

/* Function Declarations */
extern void kf_oldx(double X[3], double P[9], const double Z[3], double U,
                    double gh, double ga, double gw, double T);
extern void kf_oldx_initialize(void);
extern void kf_oldx_terminate(void);

#endif

/*
 * File trailer for kf_oldx.h
 *
 * [EOF]
 */
