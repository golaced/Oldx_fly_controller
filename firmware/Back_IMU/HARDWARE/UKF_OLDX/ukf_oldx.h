/*
 * File: ukf_oldx.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 31-Oct-2016 09:13:51
 */

#ifndef __UKF_OLDX_H__
#define __UKF_OLDX_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "ukf_oldx_types.h"

/* Function Declarations */
extern void ukf_oldx( double ffun[36], double X[6], double P[36], 
                     double hfun[36],  double Z[6],  double Q[36],
                      double R[36]);
extern void ukf_oldx_initialize(void);
extern void ukf_oldx_terminate(void);

#endif

/*
 * File trailer for ukf_oldx.h
 *
 * [EOF]
 */
