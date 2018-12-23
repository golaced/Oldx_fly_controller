/*
 * File: ukf_oldx_all.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 01-Nov-2016 08:55:36
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "ukf_oldx_all.h"

/* Function Declarations */
static void diag(const double v[13], double d[169]);
static void inv(const double x[36], double y[36]);
static void invNxN(const double x[36], double y[36]);

/* Function Definitions */

/*
 * Arguments    : const double v[13]
 *                double d[169]
 * Return Type  : void
 */
static void diag(const double v[13], double d[169])
{
  int j;
  memset(&d[0L], 0, 169U * sizeof(double));
  for (j = 0; j < 13; j++) {
    d[j + 13 * j] = v[j];
  }
}

/*
 * Arguments    : const double x[36]
 *                double y[36]
 * Return Type  : void
 */
static void inv(const double x[36], double y[36])
{
  invNxN(x, y);
}

/*
 * Arguments    : const double x[36]
 *                double y[36]
 * Return Type  : void
 */
static void invNxN(const double x[36], double y[36])
{
  double A[36];
  int i0;
  int ipiv[6];
  int j;
  int c;
  int jBcol;
  int ix;
  double smax;
  int k;
  double s;
  int i;
  int kAcol;
  int p[6];
  for (i0 = 0; i0 < 36; i0++) {
    y[i0] = 0.0;
    A[i0] = x[i0];
  }

  for (i0 = 0; i0 < 6; i0++) {
    ipiv[i0] = 1 + i0;
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    jBcol = 0;
    ix = c;
    smax = fabs(A[c]);
    for (k = 2; k <= 6 - j; k++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        jBcol = k - 1;
        smax = s;
      }
    }

    if (A[c + jBcol] != 0.0) {
      if (jBcol != 0) {
        ipiv[j] = (j + jBcol) + 1;
        ix = j;
        jBcol += j;
        for (k = 0; k < 6; k++) {
          smax = A[ix];
          A[ix] = A[jBcol];
          A[jBcol] = smax;
          ix += 6;
          jBcol += 6;
        }
      }

      i0 = (c - j) + 6;
      for (i = c + 1; i + 1 <= i0; i++) {
        A[i] /= A[c];
      }
    }

    jBcol = c;
    kAcol = c + 6;
    for (i = 1; i <= 5 - j; i++) {
      smax = A[kAcol];
      if (A[kAcol] != 0.0) {
        ix = c + 1;
        i0 = (jBcol - j) + 12;
        for (k = 7 + jBcol; k + 1 <= i0; k++) {
          A[k] += A[ix] * -smax;
          ix++;
        }
      }

      kAcol += 6;
      jBcol += 6;
    }
  }

  for (i0 = 0; i0 < 6; i0++) {
    p[i0] = 1 + i0;
  }

  for (k = 0; k < 5; k++) {
    if (ipiv[k] > 1 + k) {
      jBcol = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = jBcol;
    }
  }

  for (k = 0; k < 6; k++) {
    c = p[k] - 1;
    y[k + 6 * (p[k] - 1)] = 1.0;
    for (j = k; j + 1 < 7; j++) {
      if (y[j + 6 * c] != 0.0) {
        for (i = j + 1; i + 1 < 7; i++) {
          y[i + 6 * c] -= y[j + 6 * c] * A[i + 6 * j];
        }
      }
    }
  }

  for (j = 0; j < 6; j++) {
    jBcol = 6 * j;
    for (k = 5; k > -1; k += -1) {
      kAcol = 6 * k;
      if (y[k + jBcol] != 0.0) {
        y[k + jBcol] /= A[k + kAcol];
        for (i = 0; i + 1 <= k; i++) {
          y[i + jBcol] -= y[k + jBcol] * A[i + kAcol];
        }
      }
    }
  }
}

/*
 * Arguments    : const double ffun[36]
 *                double X[6]
 *                double P[36]
 *                const double hfun[36]
 *                const double Z[6]
 *                const double Q[36]
 *                const double R[36]
 * Return Type  : void
 */
void ukf_oldx_all(const double ffun[36], double X[6], double P[36], const double
                  hfun[36], const double Z[6], const double Q[36], const double
                  R[36])
{
  double Wc[13];
  int i1;
  static const double Wm[13] = { -9998.999999993699, 833.33333333280837,
    833.33333333280837, 833.33333333280837, 833.33333333280837,
    833.33333333280837, 833.33333333280837, 833.33333333280837,
    833.33333333280837, 833.33333333280837, 833.33333333280837,
    833.33333333280837, 833.33333333280837 };

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
  int i2;
  int i;
  double b_X[78];
  double A[36];
  double Y[36];
  double X1means[6];
  double X1[78];
  static const double dv0[13] = { -9998.999999993699, 833.33333333280837,
    833.33333333280837, 833.33333333280837, 833.33333333280837,
    833.33333333280837, 833.33333333280837, 833.33333333280837,
    833.33333333280837, 833.33333333280837, 833.33333333280837,
    833.33333333280837, 833.33333333280837 };

  double X2[78];
  double b[169];
  double Zpre[6];
  double Z1[78];
  double b_b[169];
  double c_b[169];
  double b_X1[36];
  double b_Z[6];
  double K[36];

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  for (i1 = 0; i1 < 13; i1++) {
    Wc[i1] = Wm[i1];
  }

  Wc[0] = -9996.0000999936983;
  info = 0;
  colj = 0;
  j = 1;
  exitg1 = false;
  while ((!exitg1) && (j < 7)) {
    jj = (colj + j) - 1;
    ajj = 0.0;
    if (j - 1 < 1) {
    } else {
      ix = colj;
      iy = colj;
      for (jmax = 1; jmax < j; jmax++) {
        ajj += P[ix] * P[iy];
        ix++;
        iy++;
      }
    }

    ajj = P[jj] - ajj;
    if (ajj > 0.0) {
      ajj = sqrt(ajj);
      P[jj] = ajj;
      if (j < 6) {
        if (j - 1 == 0) {
        } else {
          iy = jj + 6;
          i1 = (colj + 6 * (5 - j)) + 7;
          for (jmax = colj + 7; jmax <= i1; jmax += 6) {
            ix = colj;
            c = 0.0;
            i2 = (jmax + j) - 2;
            for (i = jmax; i <= i2; i++) {
              c += P[i - 1] * P[ix];
              ix++;
            }

            P[iy] += -c;
            iy += 6;
          }
        }

        ajj = 1.0 / ajj;
        i1 = (jj + 6 * (5 - j)) + 7;
        for (jmax = jj + 6; jmax + 1 <= i1; jmax += 6) {
          P[jmax] *= ajj;
        }

        colj += 6;
      }

      j++;
    } else {
      P[jj] = ajj;
      info = j;
      exitg1 = true;
    }
  }

  if (info == 0) {
    jmax = 6;
  } else {
    jmax = info - 1;
  }

  for (j = 0; j + 1 <= jmax; j++) {
    for (i = j + 1; i + 1 <= jmax; i++) {
      P[i + 6 * j] = 0.0;
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      A[i2 + 6 * i1] = 0.024494897427839498 * P[i1 + 6 * i2];
      Y[i2 + 6 * i1] = X[i2];
    }

    X1means[i1] = 0.0;
    b_X[i1] = X[i1];
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      b_X[i2 + 6 * (i1 + 1)] = Y[i2 + 6 * i1] + A[i2 + 6 * i1];
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      b_X[i2 + 6 * (i1 + 7)] = Y[i2 + 6 * i1] - A[i2 + 6 * i1];
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 13; i2++) {
      X1[i1 + 6 * i2] = 0.0;
      for (jmax = 0; jmax < 6; jmax++) {
        X1[i1 + 6 * i2] += ffun[i1 + 6 * jmax] * b_X[jmax + 6 * i2];
      }
    }
  }

  for (jmax = 0; jmax < 13; jmax++) {
    /*  Xsigma_pre(:,k)=fun(Xsigma(:,k)); */
    for (i1 = 0; i1 < 6; i1++) {
      X1means[i1] += dv0[jmax] * X1[i1 + 6 * jmax];
    }
  }

  for (i1 = 0; i1 < 13; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      X2[i2 + 6 * i1] = X1[i2 + 6 * i1] - X1means[i2];
    }
  }

  diag(Wc, b);
  for (i = 0; i < 6; i++) {
    Zpre[i] = 0.0;
    for (i1 = 0; i1 < 13; i1++) {
      Z1[i + 6 * i1] = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        Z1[i + 6 * i1] += hfun[i + 6 * i2] * X1[i2 + 6 * i1];
      }
    }
  }

  for (jmax = 0; jmax < 13; jmax++) {
    /*  Xsigma_pre(:,k)=fun(Xsigma(:,k)); */
    for (i1 = 0; i1 < 6; i1++) {
      Zpre[i1] += dv0[jmax] * Z1[i1 + 6 * jmax];
    }
  }

  for (i1 = 0; i1 < 13; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      X1[i2 + 6 * i1] = Z1[i2 + 6 * i1] - Zpre[i2];
    }
  }

  diag(Wc, b_b);
  diag(Wc, c_b);
  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 13; i2++) {
      b_X[i1 + 6 * i2] = 0.0;
      for (jmax = 0; jmax < 13; jmax++) {
        b_X[i1 + 6 * i2] += X2[i1 + 6 * jmax] * c_b[jmax + 13 * i2];
      }
    }

    for (i2 = 0; i2 < 6; i2++) {
      A[i1 + 6 * i2] = 0.0;
      for (jmax = 0; jmax < 13; jmax++) {
        A[i1 + 6 * i2] += b_X[i1 + 6 * jmax] * X1[i2 + 6 * jmax];
      }
    }

    for (i2 = 0; i2 < 13; i2++) {
      Z1[i1 + 6 * i2] = 0.0;
      for (jmax = 0; jmax < 13; jmax++) {
        Z1[i1 + 6 * i2] += X1[i1 + 6 * jmax] * b_b[jmax + 13 * i2];
      }
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      ajj = 0.0;
      for (jmax = 0; jmax < 13; jmax++) {
        ajj += Z1[i1 + 6 * jmax] * X1[i2 + 6 * jmax];
      }

      b_X1[i1 + 6 * i2] = ajj + R[i1 + 6 * i2];
    }
  }

  inv(b_X1, Y);
  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      K[i1 + 6 * i2] = 0.0;
      for (jmax = 0; jmax < 6; jmax++) {
        K[i1 + 6 * i2] += A[i1 + 6 * jmax] * Y[jmax + 6 * i2];
      }
    }

    b_Z[i1] = Z[i1] - Zpre[i1];
  }

  for (i1 = 0; i1 < 6; i1++) {
    ajj = 0.0;
    for (i2 = 0; i2 < 6; i2++) {
      ajj += K[i1 + 6 * i2] * b_Z[i2];
    }

    X[i1] = X1means[i1] + ajj;
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 13; i2++) {
      b_X[i1 + 6 * i2] = 0.0;
      for (jmax = 0; jmax < 13; jmax++) {
        b_X[i1 + 6 * i2] += X2[i1 + 6 * jmax] * b[jmax + 13 * i2];
      }
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      ajj = 0.0;
      for (jmax = 0; jmax < 13; jmax++) {
        ajj += b_X[i1 + 6 * jmax] * X2[i2 + 6 * jmax];
      }

      Y[i1 + 6 * i2] = ajj + Q[i1 + 6 * i2];
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      b_X1[i1 + 6 * i2] = 0.0;
      for (jmax = 0; jmax < 6; jmax++) {
        b_X1[i1 + 6 * i2] += K[i1 + 6 * jmax] * A[i2 + 6 * jmax];
      }
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      P[i2 + 6 * i1] = Y[i2 + 6 * i1] - b_X1[i2 + 6 * i1];
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ukf_oldx_all_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ukf_oldx_all_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for ukf_oldx_all.c
 *
 * [EOF]
 */
