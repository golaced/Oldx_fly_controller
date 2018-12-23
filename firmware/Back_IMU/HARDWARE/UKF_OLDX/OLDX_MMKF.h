/*
 * File: OLDX_MMKF.h
 *
 * MATLAB Coder version            : 3.0
 * C/C++ source code generated on  : 04-Mar-2018 21:09:14
 */

#ifndef __OLDX_MMKF_H__
#define __OLDX_MMKF_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "OLDX_MMKF_types.h"

/* Function Declarations */
extern void OLDX_MMKF(double X[3], double X_B[60], double K_B[45], double C[9],
                      double E_B[15], double P[9], double Z[3], double U, const
                      double A[9], const double B[3], double H[9], const double Q[9],
                      const double R[9], const double Param[8]);
extern void OLDX_MMKF_initialize(void);
extern void OLDX_MMKF_terminate(void);

#endif

/*
 * File trailer for OLDX_MMKF.h
 *
 * [EOF]
 */
