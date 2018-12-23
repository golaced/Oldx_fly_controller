/*
 * File: ukf_nav.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 01-Nov-2016 16:06:13
 */

#ifndef __UKF_NAV_H__
#define __UKF_NAV_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "ukf_nav_types.h"

/* Function Declarations */
extern void ukf_nav(const double ffun[81], double X[9], double P[81], const
                    double hfun[81], const double Z[9], const double Q[81],
                    const double R[81]);
extern void ukf_nav_initialize(void);
extern void ukf_nav_terminate(void);

#endif

/*
 * File trailer for ukf_nav.h
 *
 * [EOF]
 */
