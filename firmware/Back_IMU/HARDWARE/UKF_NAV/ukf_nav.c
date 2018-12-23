/*
 * File: ukf_nav.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 01-Nov-2016 16:06:13
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "ukf_nav.h"

/* Function Declarations */
static void diag(const double v[19], double d[361]);
static void inv(const double x[81], double y[81]);
static void invNxN(const double x[81], double y[81]);
static void sigmas(const double X[9], const double P[81], double c, double Xset
                   [171]);
static void ut(const double fun[81], const double Xsigma[171], const double Wm
               [19], const double Wc[19], const double COV[81], double Xmeans[9],
               double Xsigma_pre[171], double P[81], double Xdiv[171]);

/* Function Definitions */

/*
 * Arguments    : const double v[19]
 *                double d[361]
 * Return Type  : void
 */
static void diag(const double v[19], double d[361])
{
  int j;
  memset(&d[0L], 0, 361U * sizeof(double));
  for (j = 0; j < 19; j++) {
    d[j + 19 * j] = v[j];
  }
}

/*
 * Arguments    : const double x[81]
 *                double y[81]
 * Return Type  : void
 */
static void inv(const double x[81], double y[81])
{
  invNxN(x, y);
}

/*
 * Arguments    : const double x[81]
 *                double y[81]
 * Return Type  : void
 */
static void invNxN(const double x[81], double y[81])
{
  double A[81];
  int i3;
  int ipiv[9];
  int j;
  int c;
  int jBcol;
  int ix;
  double smax;
  int k;
  double s;
  int i;
  int kAcol;
  int p[9];
  for (i3 = 0; i3 < 81; i3++) {
    y[i3] = 0.0;
    A[i3] = x[i3];
  }

  for (i3 = 0; i3 < 9; i3++) {
    ipiv[i3] = 1 + i3;
  }

  for (j = 0; j < 8; j++) {
    c = j * 10;
    jBcol = 0;
    ix = c;
    smax = fabs(A[c]);
    for (k = 2; k <= 9 - j; k++) {
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
        for (k = 0; k < 9; k++) {
          smax = A[ix];
          A[ix] = A[jBcol];
          A[jBcol] = smax;
          ix += 9;
          jBcol += 9;
        }
      }

      i3 = (c - j) + 9;
      for (i = c + 1; i + 1 <= i3; i++) {
        A[i] /= A[c];
      }
    }

    jBcol = c;
    kAcol = c + 9;
    for (i = 1; i <= 8 - j; i++) {
      smax = A[kAcol];
      if (A[kAcol] != 0.0) {
        ix = c + 1;
        i3 = (jBcol - j) + 18;
        for (k = 10 + jBcol; k + 1 <= i3; k++) {
          A[k] += A[ix] * -smax;
          ix++;
        }
      }

      kAcol += 9;
      jBcol += 9;
    }
  }

  for (i3 = 0; i3 < 9; i3++) {
    p[i3] = 1 + i3;
  }

  for (k = 0; k < 8; k++) {
    if (ipiv[k] > 1 + k) {
      jBcol = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = jBcol;
    }
  }

  for (k = 0; k < 9; k++) {
    c = p[k] - 1;
    y[k + 9 * (p[k] - 1)] = 1.0;
    for (j = k; j + 1 < 10; j++) {
      if (y[j + 9 * c] != 0.0) {
        for (i = j + 1; i + 1 < 10; i++) {
          y[i + 9 * c] -= y[j + 9 * c] * A[i + 9 * j];
        }
      }
    }
  }

  for (j = 0; j < 9; j++) {
    jBcol = 9 * j;
    for (k = 8; k > -1; k += -1) {
      kAcol = 9 * k;
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
 * Arguments    : const double X[9]
 *                const double P[81]
 *                double c
 *                double Xset[171]
 * Return Type  : void
 */
static void sigmas(const double X[9], const double P[81], double c, double Xset
                   [171])
{
  double Y[81];
  int info;
  int colj;
  int j;
  boolean_T exitg1;
  int jj;
  double ajj;
  int ix;
  int iy;
  int jmax;
  int i0;
  int i;
  double b_c;
  int ia;
  double A[81];

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  memcpy(&Y[0L], &P[0L], 81U * sizeof(double));
  info = 0;
  colj = 0;
  j = 1;
  exitg1 = false;
  while ((!exitg1) && (j < 10)) {
    jj = (colj + j) - 1;
    ajj = 0.0;
    if (j - 1 < 1) {
    } else {
      ix = colj;
      iy = colj;
      for (jmax = 1; jmax < j; jmax++) {
        ajj += Y[ix] * Y[iy];
        ix++;
        iy++;
      }
    }

    ajj = Y[jj] - ajj;
    if (ajj > 0.0) {
      ajj = sqrt(ajj);
      Y[jj] = ajj;
      if (j < 9) {
        if (j - 1 == 0) {
        } else {
          iy = jj + 9;
          i0 = (colj + 9 * (8 - j)) + 10;
          for (i = colj + 10; i <= i0; i += 9) {
            ix = colj;
            b_c = 0.0;
            jmax = (i + j) - 2;
            for (ia = i; ia <= jmax; ia++) {
              b_c += Y[ia - 1] * Y[ix];
              ix++;
            }

            Y[iy] += -b_c;
            iy += 9;
          }
        }

        ajj = 1.0 / ajj;
        i0 = (jj + 9 * (8 - j)) + 10;
        for (jmax = jj + 9; jmax + 1 <= i0; jmax += 9) {
          Y[jmax] *= ajj;
        }

        colj += 9;
      }

      j++;
    } else {
      Y[jj] = ajj;
      info = j;
      exitg1 = true;
    }
  }

  if (info == 0) {
    jmax = 9;
  } else {
    jmax = info - 1;
  }

  for (j = 0; j + 1 <= jmax; j++) {
    for (i = j + 1; i + 1 <= jmax; i++) {
      Y[i + 9 * j] = 0.0;
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    for (jmax = 0; jmax < 9; jmax++) {
      A[jmax + 9 * i0] = c * Y[i0 + 9 * jmax];
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    for (jmax = 0; jmax < 9; jmax++) {
      Y[jmax + 9 * i0] = X[jmax];
    }

    Xset[i0] = X[i0];
  }

  for (i0 = 0; i0 < 9; i0++) {
    for (jmax = 0; jmax < 9; jmax++) {
      Xset[jmax + 9 * (i0 + 1)] = Y[jmax + 9 * i0] + A[jmax + 9 * i0];
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    for (jmax = 0; jmax < 9; jmax++) {
      Xset[jmax + 9 * (i0 + 10)] = Y[jmax + 9 * i0] - A[jmax + 9 * i0];
    }
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
}

/*
 * Arguments    : const double fun[81]
 *                const double Xsigma[171]
 *                const double Wm[19]
 *                const double Wc[19]
 *                const double COV[81]
 *                double Xmeans[9]
 *                double Xsigma_pre[171]
 *                double P[81]
 *                double Xdiv[171]
 * Return Type  : void
 */
static void ut(const double fun[81], const double Xsigma[171], const double Wm
               [19], const double Wc[19], const double COV[81], double Xmeans[9],
               double Xsigma_pre[171], double P[81], double Xdiv[171])
{
  int i;
  int i1;
  int i2;
  double d[361];
  double b_Xdiv[171];
  double d0;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  for (i = 0; i < 9; i++) {
    Xmeans[i] = 0.0;
    for (i1 = 0; i1 < 19; i1++) {
      Xsigma_pre[i + 9 * i1] = 0.0;
      for (i2 = 0; i2 < 9; i2++) {
        Xsigma_pre[i + 9 * i1] += fun[i + 9 * i2] * Xsigma[i2 + 9 * i1];
      }
    }
  }

  for (i = 0; i < 19; i++) {
    /*  Xsigma_pre(:,k)=fun(Xsigma(:,k)); */
    for (i1 = 0; i1 < 9; i1++) {
      Xmeans[i1] += Wm[i] * Xsigma_pre[i1 + 9 * i];
    }
  }

  for (i1 = 0; i1 < 19; i1++) {
    for (i2 = 0; i2 < 9; i2++) {
      Xdiv[i2 + 9 * i1] = Xsigma_pre[i2 + 9 * i1] - Xmeans[i2];
    }
  }

  memset(&d[0L], 0, 361U * sizeof(double));
  for (i = 0; i < 19; i++) {
    d[i + 19 * i] = Wc[i];
  }

  for (i1 = 0; i1 < 9; i1++) {
    for (i2 = 0; i2 < 19; i2++) {
      b_Xdiv[i1 + 9 * i2] = 0.0;
      for (i = 0; i < 19; i++) {
        b_Xdiv[i1 + 9 * i2] += Xdiv[i1 + 9 * i] * d[i + 19 * i2];
      }
    }
  }

  for (i1 = 0; i1 < 9; i1++) {
    for (i2 = 0; i2 < 9; i2++) {
      d0 = 0.0;
      for (i = 0; i < 19; i++) {
        d0 += b_Xdiv[i1 + 9 * i] * Xdiv[i2 + 9 * i];
      }

      P[i1 + 9 * i2] = d0 + COV[i1 + 9 * i2];
    }
  }
}

/*
 * Arguments    : const double ffun[81]
 *                double X[9]
 *                double P[81]
 *                const double hfun[81]
 *                const double Z[9]
 *                const double Q[81]
 *                const double R[81]
 * Return Type  : void
 */
void ukf_nav(const double ffun[81], double X[9], double P[81], const double
             hfun[81], const double Z[9], const double Q[81], const double R[81])
{
  double Wc[19];
  static const double Wm[19] = { -9999.0000000035689, 555.55555555575381,
    555.55555555575381, 555.55555555575381, 555.55555555575381,
    555.55555555575381, 555.55555555575381, 555.55555555575381,
    555.55555555575381, 555.55555555575381, 555.55555555575381,
    555.55555555575381, 555.55555555575381, 555.55555555575381,
    555.55555555575381, 555.55555555575381, 555.55555555575381,
    555.55555555575381, 555.55555555575381 };

  double dv0[171];
  double X2[171];
  double P1[81];
  double X1[171];
  double X1means[9];
  double Z2[171];
  double K[81];
  double Xsigmaset[171];
  double Zpre[9];
  double b[361];
  double b_b[81];
  double b_Z[9];
  int i4;
  int i5;
  int i6;
  double Pxz[81];
  double d1;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  memcpy(&Wc[0L], &Wm[0L], 19U * sizeof(double));
  Wc[0] = -9996.0001000035681;
  sigmas(X, P, 0.029999999999994646, dv0);
  ut(ffun, dv0, Wm, Wc, Q, X1means, X1, P1, X2);
  ut(hfun, X1, Wm, Wc, R, Zpre, Xsigmaset, K, Z2);
  diag(Wc, b);
  inv(K, b_b);
  for (i4 = 0; i4 < 9; i4++) {
    for (i5 = 0; i5 < 19; i5++) {
      X1[i4 + 9 * i5] = 0.0;
      for (i6 = 0; i6 < 19; i6++) {
        X1[i4 + 9 * i5] += X2[i4 + 9 * i6] * b[i6 + 19 * i5];
      }
    }

    for (i5 = 0; i5 < 9; i5++) {
      Pxz[i4 + 9 * i5] = 0.0;
      for (i6 = 0; i6 < 19; i6++) {
        Pxz[i4 + 9 * i5] += X1[i4 + 9 * i6] * Z2[i5 + 9 * i6];
      }
    }

    for (i5 = 0; i5 < 9; i5++) {
      K[i4 + 9 * i5] = 0.0;
      for (i6 = 0; i6 < 9; i6++) {
        K[i4 + 9 * i5] += Pxz[i4 + 9 * i6] * b_b[i6 + 9 * i5];
      }
    }

    b_Z[i4] = Z[i4] - Zpre[i4];
  }

  for (i4 = 0; i4 < 9; i4++) {
    d1 = 0.0;
    for (i5 = 0; i5 < 9; i5++) {
      d1 += K[i4 + 9 * i5] * b_Z[i5];
    }

    X[i4] = X1means[i4] + d1;
  }

  for (i4 = 0; i4 < 9; i4++) {
    for (i5 = 0; i5 < 9; i5++) {
      d1 = 0.0;
      for (i6 = 0; i6 < 9; i6++) {
        d1 += K[i4 + 9 * i6] * Pxz[i5 + 9 * i6];
      }

      P[i4 + 9 * i5] = P1[i4 + 9 * i5] - d1;
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ukf_nav_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ukf_nav_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for ukf_nav.c
 *
 * [EOF]
 */
