/*
 * File: ukf_oldx_baro.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 01-Nov-2016 11:33:29
 */

#ifndef __UKF_OLDX_BARO_H__
#define __UKF_OLDX_BARO_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "ukf_oldx_baro_types.h"

/* Function Declarations */
extern void ukf_oldx_baro(const double ffunr[9], double Xr[3], double Pr[9],
  const double hfunr[9], const double Zr[3], const double Qr[9], const double
  Rr[9]);
extern void ukf_oldx_baro_initialize(void);
extern void ukf_oldx_baro_terminate(void);

#endif

/*
 * File trailer for ukf_oldx_baro.h
 *
 * [EOF]
 */
