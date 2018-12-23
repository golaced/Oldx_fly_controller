/*
 * File: kf_oldx.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Nov-2016 14:12:33
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "kf_oldx.h"

/* Function Declarations */
static void inv(const double x[9], double y[9]);
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : const double x[9]
 *                double y[9]
 * Return Type  : void
 */
static void inv(const double x[9], double y[9])
{
  double b_x[9];
  int p1;
  int p2;
  int p3;
  double absx11;
  double absx21;
  double absx31;
  int itmp;
  double b_y;
  for (p1 = 0; p1 < 9; p1++) {
    b_x[p1] = x[p1];
  }

  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = fabs(x[0]);
  absx21 = fabs(x[1]);
  absx31 = fabs(x[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = x[1];
    b_x[1] = x[0];
    b_x[3] = x[4];
    b_x[4] = x[3];
    b_x[6] = x[7];
    b_x[7] = x[6];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      b_x[0] = x[2];
      b_x[2] = x[0];
      b_x[3] = x[5];
      b_x[5] = x[3];
      b_x[6] = x[8];
      b_x[8] = x[6];
    }
  }

  absx21 = b_x[1] / b_x[0];
  b_x[1] /= b_x[0];
  absx11 = b_x[2] / b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= absx21 * b_x[3];
  b_x[5] -= absx11 * b_x[3];
  b_x[7] -= absx21 * b_x[6];
  b_x[8] -= absx11 * b_x[6];
  if (fabs(b_x[5]) > fabs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    b_x[1] = absx11;
    b_x[2] = absx21;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }

  absx31 = b_x[5];
  b_y = b_x[4];
  absx21 = b_x[5] / b_x[4];
  b_x[8] -= absx21 * b_x[7];
  absx11 = (absx21 * b_x[1] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -(absx31 / b_y) / b_x[8];
  absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
  y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p2 + 1] = absx21;
  y[p2 + 2] = absx11;
  absx11 = 1.0 / b_x[8];
  absx21 = -b_x[7] * absx11 / b_x[4];
  y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = rtNaN;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * Arguments    : double X[3]
 *                double P[9]
 *                const double Z[3]
 *                double U
 *                double gh
 *                double ga
 *                double gw
 *                double T
 * Return Type  : void
 */
void kf_oldx(double X[3], double P[9], const double Z[3], double U, double gh,
             double ga, double gw, double T)
{
  double A[9];
  int i0;
  static const int iv0[3] = { 0, 0, 1 };

  double c;
  double b_c;
  double b_T[3];
  double d2;
  int i1;
  double X_pre[3];
  double b_A[9];
  double b_gh[9];
  int i2;
  double b[9];
  double P_pre[9];
  static const int b_b[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const double dv0[3] = { 0.0, 1.0E-5, 0.0 };

  static const double dv1[3] = { 0.0, 0.0, 1.0E-5 };

  double b_P_pre[9];
  double c_b[3];
  double d_b[3];
  double K[9];
  static const int I[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  A[0] = 1.0;
  A[3] = T;
  A[6] = -T * T / 2.0;
  A[1] = 0.0;
  A[4] = 1.0;
  A[7] = -T;
  for (i0 = 0; i0 < 3; i0++) {
    A[2 + 3 * i0] = iv0[i0];
  }

  c = ga * ga;
  b_c = gw * gw;
  b_T[0] = T * T / 2.0;
  b_T[1L] = T;
  b_T[2L] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    d2 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d2 += A[i0 + 3 * i1] * X[i1];
    }

    X_pre[i0] = d2 + b_T[i0] * U;
  }

  b_gh[0] = 0.25 * rt_powd_snf(T, 4.0);
  b_gh[3] = 0.5 * rt_powd_snf(T, 3.0);
  b_gh[6] = 0.0;
  b_gh[1] = 0.5 * rt_powd_snf(T, 3.0);
  b_gh[4] = T * T;
  b_gh[7] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      b_A[i0 + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        b_A[i0 + 3 * i1] += A[i0 + 3 * i2] * P[i2 + 3 * i1];
      }
    }

    b_gh[2 + 3 * i0] = 0.0;
  }

  b[0] = 0.1111111111111111 * rt_powd_snf(T, 6.0);
  b[3] = 0.16666666666666666 * rt_powd_snf(T, 5.0);
  b[6] = -0.33333333333333331 * rt_powd_snf(T, 4.0);
  b[1] = 0.16666666666666666 * rt_powd_snf(T, 5.0);
  b[4] = 0.25 * rt_powd_snf(T, 4.0);
  b[7] = -0.5 * (T * T);
  b[2] = -0.33333333333333331 * rt_powd_snf(T, 4.0);
  b[5] = -0.5 * rt_powd_snf(T, 3.0);
  b[8] = T * T;
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      d2 = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        d2 += b_A[i0 + 3 * i2] * A[i1 + 3 * i2];
      }

      P_pre[i0 + 3 * i1] = d2 + (b_gh[i0 + 3 * i1] * c + b[i0 + 3 * i1] * b_c);
    }
  }

  b_gh[0] = gh + 1.0E-5;
  b_gh[3] = 0.0;
  b_gh[6] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      b[i0 + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        b[i0 + 3 * i1] += (double)b_b[i0 + 3 * i2] * P_pre[i2 + 3 * i1];
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      b_A[i0 + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        b_A[i0 + 3 * i1] += b[i0 + 3 * i2] * (double)b_b[i2 + 3 * i1];
      }
    }

    b_gh[1 + 3 * i0] = dv0[i0];
    b_gh[2 + 3 * i0] = dv1[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      b[i1 + 3 * i0] = b_A[i1 + 3 * i0] + b_gh[i1 + 3 * i0];
      b_P_pre[i0 + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        b_P_pre[i0 + 3 * i1] += P_pre[i0 + 3 * i2] * (double)b_b[i2 + 3 * i1];
      }
    }
  }

  inv(b, A);
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      K[i0 + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        K[i0 + 3 * i1] += b_P_pre[i0 + 3 * i2] * A[i2 + 3 * i1];
      }
    }

    b_T[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      b_T[i0] += (double)b_b[i0 + 3 * i1] * Z[i1];
    }

    c_b[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      c_b[i0] += (double)b_b[i0 + 3 * i1] * X_pre[i1];
    }

    d_b[i0] = b_T[i0] - c_b[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    d2 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d2 += K[i0 + 3 * i1] * d_b[i1];
    }

    X[i0] = X_pre[i0] + d2;
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      d2 = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        d2 += K[i0 + 3 * i2] * (double)b_b[i2 + 3 * i1];
      }

      b_A[i0 + 3 * i1] = (double)I[i0 + 3 * i1] - d2;
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      P[i0 + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        P[i0 + 3 * i1] += b_A[i0 + 3 * i2] * P_pre[i2 + 3 * i1];
      }
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void kf_oldx_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void kf_oldx_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for kf_oldx.c
 *
 * [EOF]
 */
