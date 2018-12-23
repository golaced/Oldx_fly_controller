/*
 * File: ukf_oldx.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 31-Oct-2016 09:13:51
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "ukf_oldx.h"

/* Function Declarations */
static void diag( double v[13], double d[169]);
static void inv( double x[36], double y[36]);
static void invNxN( double x[36], double y[36]);
static void sigmas( double X[6],  double P[36], double c, double Xset
                   [78]);
static void ut( double fun[36],  double Xsigma[78],  double Wm[13],
                double Wc[13],  double COV[36], double Xmeans[6],
               double Xsigma_pre[78], double P[36], double Xdiv[78]);

/* Function Definitions */

/*
 * Arguments    :  double v[13]
 *                double d[169]
 * Return Type  : void
 */
static void diag( double v[13], double d[169])
{
  int j;
  memset(&d[0L], 0, 169U * sizeof(double));
  for (j = 0; j < 13; j++) {
    d[j + 13 * j] = v[j];
  }
}

/*
 * Arguments    :  double x[36]
 *                double y[36]
 * Return Type  : void
 */
static void inv( double x[36], double y[36])
{
  invNxN(x, y);
}

/*
 * Arguments    :  double x[36]
 *                double y[36]
 * Return Type  : void
 */
static void invNxN( double x[36], double y[36])
{
  double A[36];
  int i3;
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
  for (i3 = 0; i3 < 36; i3++) {
    y[i3] = 0.0;
    A[i3] = x[i3];
  }

  for (i3 = 0; i3 < 6; i3++) {
    ipiv[i3] = 1 + i3;
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

      i3 = (c - j) + 6;
      for (i = c + 1; i + 1 <= i3; i++) {
        A[i] /= A[c];
      }
    }

    jBcol = c;
    kAcol = c + 6;
    for (i = 1; i <= 5 - j; i++) {
      smax = A[kAcol];
      if (A[kAcol] != 0.0) {
        ix = c + 1;
        i3 = (jBcol - j) + 12;
        for (k = 7 + jBcol; k + 1 <= i3; k++) {
          A[k] += A[ix] * -smax;
          ix++;
        }
      }

      kAcol += 6;
      jBcol += 6;
    }
  }

  for (i3 = 0; i3 < 6; i3++) {
    p[i3] = 1 + i3;
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
 * Arguments    :  double X[6]
 *                 double P[36]
 *                double c
 *                double Xset[78]
 * Return Type  : void
 */
static void sigmas( double X[6],  double P[36], double c, double Xset
                   [78])
{
  double Y[36];
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
  double A[36];

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  memcpy(&Y[0L], &P[0L], 36U * sizeof(double));
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
        ajj += Y[ix] * Y[iy];
        ix++;
        iy++;
      }
    }

    ajj = Y[jj] - ajj;
    if (ajj > 0.0) {
      ajj = sqrt(ajj);
      Y[jj] = ajj;
      if (j < 6) {
        if (j - 1 == 0) {
        } else {
          iy = jj + 6;
          i0 = (colj + 6 * (5 - j)) + 7;
          for (i = colj + 7; i <= i0; i += 6) {
            ix = colj;
            b_c = 0.0;
            jmax = (i + j) - 2;
            for (ia = i; ia <= jmax; ia++) {
              b_c += Y[ia - 1] * Y[ix];
              ix++;
            }

            Y[iy] += -b_c;
            iy += 6;
          }
        }

        ajj = 1.0 / ajj;
        i0 = (jj + 6 * (5 - j)) + 7;
        for (jmax = jj + 6; jmax + 1 <= i0; jmax += 6) {
          Y[jmax] *= ajj;
        }

        colj += 6;
      }

      j++;
    } else {
      Y[jj] = ajj;
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
      Y[i + 6 * j] = 0.0;
    }
  }

  for (i0 = 0; i0 < 6; i0++) {
    for (jmax = 0; jmax < 6; jmax++) {
      A[jmax + 6 * i0] = c * Y[i0 + 6 * jmax];
    }
  }

  for (i0 = 0; i0 < 6; i0++) {
    for (jmax = 0; jmax < 6; jmax++) {
      Y[jmax + 6 * i0] = X[jmax];
    }

    Xset[i0] = X[i0];
  }

  for (i0 = 0; i0 < 6; i0++) {
    for (jmax = 0; jmax < 6; jmax++) {
      Xset[jmax + 6 * (i0 + 1)] = Y[jmax + 6 * i0] + A[jmax + 6 * i0];
    }
  }

  for (i0 = 0; i0 < 6; i0++) {
    for (jmax = 0; jmax < 6; jmax++) {
      Xset[jmax + 6 * (i0 + 7)] = Y[jmax + 6 * i0] - A[jmax + 6 * i0];
    }
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
}

/*
 * Arguments    :  double fun[36]
 *                 double Xsigma[78]
 *                 double Wm[13]
 *                 double Wc[13]
 *                 double COV[36]
 *                double Xmeans[6]
 *                double Xsigma_pre[78]
 *                double P[36]
 *                double Xdiv[78]
 * Return Type  : void
 */
static void ut( double fun[36],  double Xsigma[78],  double Wm[13],
                double Wc[13],  double COV[36], double Xmeans[6],
               double Xsigma_pre[78], double P[36], double Xdiv[78])
{
  int i;
  int i1;
  int i2;
  double d[169];
  double b_Xdiv[78];
  double d0;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  for (i = 0; i < 6; i++) {
    Xmeans[i] = 0.0;
    for (i1 = 0; i1 < 13; i1++) {
      Xsigma_pre[i + 6 * i1] = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        Xsigma_pre[i + 6 * i1] += fun[i + 6 * i2] * Xsigma[i2 + 6 * i1];
      }
    }
  }

  for (i = 0; i < 13; i++) {
    /*  Xsigma_pre(:,k)=fun(Xsigma(:,k)); */
    for (i1 = 0; i1 < 6; i1++) {
      Xmeans[i1] += Wm[i] * Xsigma_pre[i1 + 6 * i];
    }
  }

  for (i1 = 0; i1 < 13; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      Xdiv[i2 + 6 * i1] = Xsigma_pre[i2 + 6 * i1] - Xmeans[i2];
    }
  }

  memset(&d[0L], 0, 169U * sizeof(double));
  for (i = 0; i < 13; i++) {
    d[i + 13 * i] = Wc[i];
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 13; i2++) {
      b_Xdiv[i1 + 6 * i2] = 0.0;
      for (i = 0; i < 13; i++) {
        b_Xdiv[i1 + 6 * i2] += Xdiv[i1 + 6 * i] * d[i + 13 * i2];
      }
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      d0 = 0.0;
      for (i = 0; i < 13; i++) {
        d0 += b_Xdiv[i1 + 6 * i] * Xdiv[i2 + 6 * i];
      }

      P[i1 + 6 * i2] = d0 + COV[i1 + 6 * i2];
    }
  }
}

/*
 * Arguments    :  double ffun[36]
 *                double X[6]
 *                double P[36]
 *                 double hfun[36]
 *                 double Z[6]
 *                 double Q[36]
 *                 double R[36]
 * Return Type  : void
 */
void ukf_oldx( double ffun[36], double X[6], double P[36],  double
              hfun[36],  double Z[6],  double Q[36],  double R[36])
{
  double Wc[13];
  int i4;
  static  double Wm[13] = { -9998.999999993699, 833.33333333280837,
    833.33333333280837, 833.33333333280837, 833.33333333280837,
    833.33333333280837, 833.33333333280837, 833.33333333280837,
    833.33333333280837, 833.33333333280837, 833.33333333280837,
    833.33333333280837, 833.33333333280837 };

  double dv0[78];
  double X2[78];
  double P1[36];
  double X1[78];
  double X1means[6];
  double Z2[78];
  double K[36];
  double Xsigmaset[78];
  double Zpre[6];
  double b[169];
  double b_b[36];
  double b_Z[6];
  int i5;
  int i6;
  double Pxz[36];
  double d1;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  for (i4 = 0; i4 < 13; i4++) {
    Wc[i4] = Wm[i4];
  }

  Wc[0] = -9996.0000999936983;
  sigmas(X, P, 0.024494897427839498, dv0);
  ut(ffun, dv0, Wm, Wc, Q, X1means, X1, P1, X2);
  ut(hfun, X1, Wm, Wc, R, Zpre, Xsigmaset, K, Z2);
  diag(Wc, b);
  inv(K, b_b);
  for (i4 = 0; i4 < 6; i4++) {
    for (i5 = 0; i5 < 13; i5++) {
      X1[i4 + 6 * i5] = 0.0;
      for (i6 = 0; i6 < 13; i6++) {
        X1[i4 + 6 * i5] += X2[i4 + 6 * i6] * b[i6 + 13 * i5];
      }
    }

    for (i5 = 0; i5 < 6; i5++) {
      Pxz[i4 + 6 * i5] = 0.0;
      for (i6 = 0; i6 < 13; i6++) {
        Pxz[i4 + 6 * i5] += X1[i4 + 6 * i6] * Z2[i5 + 6 * i6];
      }
    }

    for (i5 = 0; i5 < 6; i5++) {
      K[i4 + 6 * i5] = 0.0;
      for (i6 = 0; i6 < 6; i6++) {
        K[i4 + 6 * i5] += Pxz[i4 + 6 * i6] * b_b[i6 + 6 * i5];
      }
    }

    b_Z[i4] = Z[i4] - Zpre[i4];
  }

  for (i4 = 0; i4 < 6; i4++) {
    d1 = 0.0;
    for (i5 = 0; i5 < 6; i5++) {
      d1 += K[i4 + 6 * i5] * b_Z[i5];
    }

    X[i4] = X1means[i4] + d1;
  }

  for (i4 = 0; i4 < 6; i4++) {
    for (i5 = 0; i5 < 6; i5++) {
      d1 = 0.0;
      for (i6 = 0; i6 < 6; i6++) {
        d1 += K[i4 + 6 * i6] * Pxz[i5 + 6 * i6];
      }

      P[i4 + 6 * i5] = P1[i4 + 6 * i5] - d1;
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ukf_oldx_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ukf_oldx_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for ukf_oldx.c
 *
 * [EOF]
 */
