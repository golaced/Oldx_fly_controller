/*
 * File: ukf_oldx_all.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 01-Nov-2016 08:55:36
 */

#ifndef __UKF_OLDX_ALL_H__
#define __UKF_OLDX_ALL_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "ukf_oldx_all_types.h"

/* Function Declarations */
extern void ukf_oldx_all(const double ffun[36], double X[6], double P[36], const
  double hfun[36], const double Z[6], const double Q[36], const double R[36]);
extern void ukf_oldx_all_initialize(void);
extern void ukf_oldx_all_terminate(void);

#endif

/*
 * File trailer for ukf_oldx_all.h
 *
 * [EOF]
 */
