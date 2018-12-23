/*
 * File: OLD_X_AVOID.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 07-Nov-2016 15:10:05
 */

#ifndef __OLD_X_AVOID_H__
#define __OLD_X_AVOID_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "OLD_X_AVOID_types.h"

/* Function Declarations */
extern void OLD_X_AVOID(const double DIS_IN[20], double A, double A_DEAD,
  unsigned int Max_try, double *x_mid, double *y_mid, double *r_mid);
extern void OLD_X_AVOID_initialize(void);
extern void OLD_X_AVOID_terminate(void);

#endif

/*
 * File trailer for OLD_X_AVOID.h
 *
 * [EOF]
 */
