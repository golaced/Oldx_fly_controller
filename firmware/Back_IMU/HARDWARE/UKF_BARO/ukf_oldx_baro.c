/*
 * File: ukf_oldx_baro.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 01-Nov-2016 11:33:29
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "ukf_oldx_baro.h"

/* Function Declarations */
static void inv(const double x[9], double y[9]);
static void ut1(const double fun1[9], const double Xsigma1[21], const double
                Wm1[7], const double Wc1[7], const double COV1[9], double
                Xmeans1[3], double Xsigma_pre1[21], double P1[9], double Xdiv1
                [21]);

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
 * Arguments    : const double fun1[9]
 *                const double Xsigma1[21]
 *                const double Wm1[7]
 *                const double Wc1[7]
 *                const double COV1[9]
 *                double Xmeans1[3]
 *                double Xsigma_pre1[21]
 *                double P1[9]
 *                double Xdiv1[21]
 * Return Type  : void
 */
static void ut1(const double fun1[9], const double Xsigma1[21], const double
                Wm1[7], const double Wc1[7], const double COV1[9], double
                Xmeans1[3], double Xsigma_pre1[21], double P1[9], double Xdiv1
                [21])
{
  int i;
  int i0;
  int i1;
  double d[49];
  double b_Xdiv1[21];
  double d0;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  for (i = 0; i < 3; i++) {
    Xmeans1[i] = 0.0;
    for (i0 = 0; i0 < 7; i0++) {
      Xsigma_pre1[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        Xsigma_pre1[i + 3 * i0] += fun1[i + 3 * i1] * Xsigma1[i1 + 3 * i0];
      }
    }
  }

  for (i = 0; i < 7; i++) {
    /*  Xsigma_pre(:,k)=fun(Xsigma(:,k)); */
    for (i0 = 0; i0 < 3; i0++) {
      Xmeans1[i0] += Wm1[i] * Xsigma_pre1[i0 + 3 * i];
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      Xdiv1[i1 + 3 * i0] = Xsigma_pre1[i1 + 3 * i0] - Xmeans1[i1];
    }
  }

  memset(&d[0L], 0, 49U * sizeof(double));
  for (i = 0; i < 7; i++) {
    d[i + 7 * i] = Wc1[i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 7; i1++) {
      b_Xdiv1[i0 + 3 * i1] = 0.0;
      for (i = 0; i < 7; i++) {
        b_Xdiv1[i0 + 3 * i1] += Xdiv1[i0 + 3 * i] * d[i + 7 * i1];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      d0 = 0.0;
      for (i = 0; i < 7; i++) {
        d0 += b_Xdiv1[i0 + 3 * i] * Xdiv1[i1 + 3 * i];
      }

      P1[i0 + 3 * i1] = d0 + COV1[i0 + 3 * i1];
    }
  }
}

/*
 * Arguments    : const double ffunr[9]
 *                double Xr[3]
 *                double Pr[9]
 *                const double hfunr[9]
 *                const double Zr[3]
 *                const double Qr[9]
 *                const double Rr[9]
 * Return Type  : void
 */
void ukf_oldx_baro(const double ffunr[9], double Xr[3], double Pr[9], const
                   double hfunr[9], const double Zr[3], const double Qr[9],
                   const double Rr[9])
{
  double Wcr[7];
  int i2;
  static const double Wmr[7] = { -9998.999999993699, 1666.6666666656167,
    1666.6666666656167, 1666.6666666656167, 1666.6666666656167,
    1666.6666666656167, 1666.6666666656167 };

  int info;
  int colj;
  int j;
  boolean_T exitg1;
  int jj;
  double ajj;
  int ix;
  int iy;
  int jmax;
  double c;
  int i3;
  int i;
  double b_Xr[21];
  double A1[9];
  double Y1[9];
  double X2r[21];
  double P1r[9];
  double X1r[21];
  double X1meansr[3];
  double Z2r[21];
  double Zprer[3];
  double d[49];
  double b_Zr[3];
  double Pxzr[9];

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  for (i2 = 0; i2 < 7; i2++) {
    Wcr[i2] = Wmr[i2];
  }

  Wcr[0] = -9996.0000999936983;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  info = 0;
  colj = 0;
  j = 1;
  exitg1 = false;
  while ((!exitg1) && (j < 4)) {
    jj = (colj + j) - 1;
    ajj = 0.0;
    if (j - 1 < 1) {
    } else {
      ix = colj;
      iy = colj;
      for (jmax = 1; jmax < j; jmax++) {
        ajj += Pr[ix] * Pr[iy];
        ix++;
        iy++;
      }
    }

    ajj = Pr[jj] - ajj;
    if (ajj > 0.0) {
      ajj = sqrt(ajj);
      Pr[jj] = ajj;
      if (j < 3) {
        if (j - 1 == 0) {
        } else {
          iy = jj + 3;
          i2 = (colj + 3 * (2 - j)) + 4;
          for (jmax = colj + 4; jmax <= i2; jmax += 3) {
            ix = colj;
            c = 0.0;
            i3 = (jmax + j) - 2;
            for (i = jmax; i <= i3; i++) {
              c += Pr[i - 1] * Pr[ix];
              ix++;
            }

            Pr[iy] += -c;
            iy += 3;
          }
        }

        ajj = 1.0 / ajj;
        i2 = (jj + 3 * (2 - j)) + 4;
        for (jmax = jj + 3; jmax + 1 <= i2; jmax += 3) {
          Pr[jmax] *= ajj;
        }

        colj += 3;
      }

      j++;
    } else {
      Pr[jj] = ajj;
      info = j;
      exitg1 = true;
    }
  }

  if (info == 0) {
    jmax = 3;
  } else {
    jmax = info - 1;
  }

  for (j = 0; j + 1 <= jmax; j++) {
    for (i = j + 1; i + 1 <= jmax; i++) {
      Pr[i + 3 * j] = 0.0;
    }
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  for (i2 = 0; i2 < 3; i2++) {
    for (i3 = 0; i3 < 3; i3++) {
      A1[i3 + 3 * i2] = 0.01732050807569423 * Pr[i2 + 3 * i3];
      Y1[i3 + 3 * i2] = Xr[i3];
    }

    b_Xr[i2] = Xr[i2];
  }

  for (i2 = 0; i2 < 3; i2++) {
    for (i3 = 0; i3 < 3; i3++) {
      b_Xr[i3 + 3 * (i2 + 1)] = Y1[i3 + 3 * i2] + A1[i3 + 3 * i2];
    }
  }

  for (i2 = 0; i2 < 3; i2++) {
    for (i3 = 0; i3 < 3; i3++) {
      b_Xr[i3 + 3 * (i2 + 4)] = Y1[i3 + 3 * i2] - A1[i3 + 3 * i2];
    }
  }

  ut1(ffunr, b_Xr, Wmr, Wcr, Qr, X1meansr, X1r, P1r, X2r);
  ut1(hfunr, X1r, Wmr, Wcr, Rr, Zprer, b_Xr, Y1, Z2r);
  memset(&d[0L], 0, 49U * sizeof(double));
  for (j = 0; j < 7; j++) {
    d[j + 7 * j] = Wcr[j];
  }

  inv(Y1, A1);
  for (i2 = 0; i2 < 3; i2++) {
    for (i3 = 0; i3 < 7; i3++) {
      b_Xr[i2 + 3 * i3] = 0.0;
      for (jmax = 0; jmax < 7; jmax++) {
        b_Xr[i2 + 3 * i3] += X2r[i2 + 3 * jmax] * d[jmax + 7 * i3];
      }
    }

    for (i3 = 0; i3 < 3; i3++) {
      Pxzr[i2 + 3 * i3] = 0.0;
      for (jmax = 0; jmax < 7; jmax++) {
        Pxzr[i2 + 3 * i3] += b_Xr[i2 + 3 * jmax] * Z2r[i3 + 3 * jmax];
      }
    }

    for (i3 = 0; i3 < 3; i3++) {
      Y1[i2 + 3 * i3] = 0.0;
      for (jmax = 0; jmax < 3; jmax++) {
        Y1[i2 + 3 * i3] += Pxzr[i2 + 3 * jmax] * A1[jmax + 3 * i3];
      }
    }

    b_Zr[i2] = Zr[i2] - Zprer[i2];
  }

  for (i2 = 0; i2 < 3; i2++) {
    ajj = 0.0;
    for (i3 = 0; i3 < 3; i3++) {
      ajj += Y1[i2 + 3 * i3] * b_Zr[i3];
    }

    Xr[i2] = X1meansr[i2] + ajj;
  }

  for (i2 = 0; i2 < 3; i2++) {
    for (i3 = 0; i3 < 3; i3++) {
      ajj = 0.0;
      for (jmax = 0; jmax < 3; jmax++) {
        ajj += Y1[i2 + 3 * jmax] * Pxzr[i3 + 3 * jmax];
      }

      Pr[i2 + 3 * i3] = P1r[i2 + 3 * i3] - ajj;
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ukf_oldx_baro_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ukf_oldx_baro_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for ukf_oldx_baro.c
 *
 * [EOF]
 */
