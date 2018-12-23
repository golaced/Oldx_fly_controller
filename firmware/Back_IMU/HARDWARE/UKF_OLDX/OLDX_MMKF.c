/*
 * File: OLDX_MMKF.c
 *
 * MATLAB Coder version            : 3.0
 * C/C++ source code generated on  : 17-Mar-2018 12:20:29
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "OLDX_MMKF.h"

/* Function Declarations */
static void inv(const double x[9], double y[9]);
static double rt_roundd_snf(double u);

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
  memcpy(&b_x[0], &x[0], 9U * sizeof(double));
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

  absx11 = b_x[1] / b_x[0];
  b_x[1] /= b_x[0];
  absx21 = b_x[2] / b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= absx11 * b_x[3];
  b_x[5] -= absx21 * b_x[3];
  b_x[7] -= absx11 * b_x[6];
  b_x[8] -= absx21 * b_x[6];
  if (fabs(b_x[5]) > fabs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    b_x[1] = absx21;
    b_x[2] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }

  absx11 = b_x[5] / b_x[4];
  b_x[5] /= b_x[4];
  b_x[8] -= absx11 * b_x[7];
  absx11 = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -b_x[5] / b_x[8];
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
 * Arguments    : double u
 * Return Type  : double
 */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * param[delay_x1 delay_x2 delay_x3 num_L en_outliers num_D2=2 out_dead=0.1 out_flt=0.386]
 * Arguments    : double X[3]
 *                double X_B[60]
 *                double K_B[45]
 *                double C[9]
 *                double E_B[15]
 *                double P[9]
 *                double Z[3]
 *                double U
 *                const double A[9]
 *                const double B[3]
 *                double H[9]
 *                const double Q[9]
 *                const double R[9]
 *                const double Param[8]
 * Return Type  : void
 */
void OLDX_MMKF(double X[3], double X_B[60], double K_B[45], double C[9], double
               E_B[15], double P[9], double Z[3], double U, const double A[9],
               const double B[3], double H[9], const double Q[9], const double
               R[9], const double Param[8])
{
  double b_A[9];
  double P_pre[9];
  int rtemp;
  double E[3];
  double maxval;
  int k;
  int r1;
  double X_pre[3];
  double b_H[3];
  short i0;
  double X_pre_delay[3];
  double c_H[3];
  double d_H[9];
  double D1[9];
  double D2[9];
  int r2;
  int r3;
  double a21;
  static const signed char I[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  /* 'OLDX_MMKF:4' I=eye(3); */
  /* 'OLDX_MMKF:5' E=zeros(3,1); */
  /* -------predict------ */
  /* 'OLDX_MMKF:7' X_pre=A*X+B*U; */
  /* X_pre=A*X+B*U */
  /* 'OLDX_MMKF:8' P_pre=A*P*A'+Q; */
  for (rtemp = 0; rtemp < 3; rtemp++) {
    E[rtemp] = 0.0;
    maxval = 0.0;
    for (k = 0; k < 3; k++) {
      maxval += A[rtemp + 3 * k] * X[k];
      b_A[rtemp + 3 * k] = 0.0;
      for (r1 = 0; r1 < 3; r1++) {
        b_A[rtemp + 3 * k] += A[rtemp + 3 * r1] * P[r1 + 3 * k];
      }
    }

    X_pre[rtemp] = maxval + B[rtemp] * U;
    for (k = 0; k < 3; k++) {
      maxval = 0.0;
      for (r1 = 0; r1 < 3; r1++) {
        maxval += b_A[rtemp + 3 * r1] * A[k + 3 * r1];
      }

      P_pre[rtemp + 3 * k] = maxval + Q[rtemp + 3 * k];
    }
  }

  /* P_pre=A*P*A'+Q */
  /* -------store------ */
  /* 'OLDX_MMKF:10' for i=20:-1:2 */
  for (rtemp = 0; rtemp < 19; rtemp++) {
    /* 'OLDX_MMKF:11' X_B(1,i)=X_B(1,i-1); */
    X_B[3 * (19 - rtemp)] = X_B[3 * (18 - rtemp)];

    /* 'OLDX_MMKF:12' X_B(2,i)=X_B(2,i-1); */
    X_B[1 + 3 * (19 - rtemp)] = X_B[1 + 3 * (18 - rtemp)];

    /* 'OLDX_MMKF:13' X_B(3,i)=X_B(3,i-1); */
    X_B[2 + 3 * (19 - rtemp)] = X_B[2 + 3 * (18 - rtemp)];
  }

  /* store k e */
  /* 'OLDX_MMKF:16' if(H(1)+H(5)+H(9)>=0) */
  if ((H[0] + H[4]) + H[8] >= 0.0) {
    /* 'OLDX_MMKF:17' for i=5:-1:2 */
    for (rtemp = 0; rtemp < 4; rtemp++) {
      /* 'OLDX_MMKF:18' K_B(:,:,i)=K_B(:,:,i-1); */
      for (k = 0; k < 3; k++) {
        for (r1 = 0; r1 < 3; r1++) {
          b_A[r1 + 3 * k] = K_B[(r1 + 3 * k) + 9 * (3 - rtemp)];
        }
      }

      /* 'OLDX_MMKF:19' E_B(:,i)=E_B(:,i-1); */
      for (k = 0; k < 3; k++) {
        for (r1 = 0; r1 < 3; r1++) {
          K_B[(r1 + 3 * k) + 9 * (4 - rtemp)] = b_A[r1 + 3 * k];
        }

        b_H[k] = E_B[k + 3 * (3 - rtemp)];
      }

      for (k = 0; k < 3; k++) {
        E_B[k + 3 * (4 - rtemp)] = b_H[k];
      }
    }
  }

  /* 'OLDX_MMKF:22' X_B(1,1)=X_pre(1); */
  X_B[0] = X_pre[0];

  /* 'OLDX_MMKF:23' X_B(2,1)=X_pre(2); */
  X_B[1] = X_pre[1];

  /* 'OLDX_MMKF:24' X_B(3,1)=X_pre(3); */
  X_B[2] = X_pre[2];

  /* -------delay handle------ */
  /* 'OLDX_MMKF:26' if(Param(1)+Param(2)+Param(3)>0) */
  if ((Param[0] + Param[1]) + Param[2] > 0.0) {
    /* 'OLDX_MMKF:27' X_pre_delay=[X_B(1,int16(Param(1)+1));X_B(2,int16(Param(2)+1));X_B(3,int16(Param(3))+1)]; */
    maxval = rt_roundd_snf(Param[0] + 1.0);
    if (maxval < 32768.0) {
      if (maxval >= -32768.0) {
        i0 = (short)maxval;
      } else {
        i0 = MIN_int16_T;
      }
    } else if (maxval >= 32768.0) {
      i0 = MAX_int16_T;
    } else {
      i0 = 0;
    }

    X_pre_delay[0] = X_B[3 * (i0 - 1)];
    maxval = rt_roundd_snf(Param[1] + 1.0);
    if (maxval < 32768.0) {
      if (maxval >= -32768.0) {
        i0 = (short)maxval;
      } else {
        i0 = MIN_int16_T;
      }
    } else if (maxval >= 32768.0) {
      i0 = MAX_int16_T;
    } else {
      i0 = 0;
    }

    X_pre_delay[1] = X_B[1 + 3 * (i0 - 1)];
    maxval = rt_roundd_snf(Param[2]);
    if (maxval < 32768.0) {
      if (maxval >= -32768.0) {
        i0 = (short)maxval;
      } else {
        i0 = MIN_int16_T;
      }
    } else if (maxval >= 32768.0) {
      i0 = MAX_int16_T;
    } else {
      i0 = 0;
    }

    k = i0 + 1;
    if (k > 32767) {
      k = 32767;
    }

    X_pre_delay[2] = X_B[2 + 3 * (k - 1)];

    /* Z_delay=H*X_pre-H*X_pre_delay; */
    /* 'OLDX_MMKF:29' Z_delay=H*X_pre-H*X_pre_delay; */
    /* 'OLDX_MMKF:30' Z=Z+Z_delay; */
    for (k = 0; k < 3; k++) {
      b_H[k] = 0.0;
      c_H[k] = 0.0;
      for (r1 = 0; r1 < 3; r1++) {
        b_H[k] += H[k + 3 * r1] * X_pre[r1];
        c_H[k] += H[k + 3 * r1] * X_pre_delay[r1];
      }

      Z[k] += b_H[k] - c_H[k];
    }
  }

  /* --------correction------ */
  /* 'OLDX_MMKF:33' if(H(1)+H(5)+H(9)>0) */
  if ((H[0] + H[4]) + H[8] > 0.0) {
    /* 'OLDX_MMKF:34' E_B(:,1)=(H*Z-H*X_pre); */
    /* -------Outliers handle--------- */
    /* 'OLDX_MMKF:36' N=Param(6); */
    /* 'OLDX_MMKF:37' dead=Param(7); */
    /* 'OLDX_MMKF:38' flt=Param(8); */
    /* 'OLDX_MMKF:39' D1=(H*P_pre*H'+R); */
    for (k = 0; k < 3; k++) {
      b_H[k] = 0.0;
      c_H[k] = 0.0;
      for (r1 = 0; r1 < 3; r1++) {
        b_H[k] += H[k + 3 * r1] * Z[r1];
        c_H[k] += H[k + 3 * r1] * X_pre[r1];
        d_H[k + 3 * r1] = 0.0;
        for (rtemp = 0; rtemp < 3; rtemp++) {
          d_H[k + 3 * r1] += H[k + 3 * rtemp] * P_pre[rtemp + 3 * r1];
        }
      }

      E_B[k] = b_H[k] - c_H[k];
      for (r1 = 0; r1 < 3; r1++) {
        maxval = 0.0;
        for (rtemp = 0; rtemp < 3; rtemp++) {
          maxval += d_H[k + 3 * rtemp] * H[r1 + 3 * rtemp];
        }

        D1[k + 3 * r1] = maxval + R[k + 3 * r1];
      }
    }

    /* 'OLDX_MMKF:40' D2=zeros(3,3); */
    memset(&D2[0], 0, 9U * sizeof(double));

    /* get estimation of D2 */
    /* 'OLDX_MMKF:42' for i=1:N+1 */
    for (rtemp = 0; rtemp < (int)(Param[5] + 1.0); rtemp++) {
      /* 'OLDX_MMKF:43' D2=D2+E_B(:,i)*E_B(:,i)'; */
      for (k = 0; k < 3; k++) {
        for (r1 = 0; r1 < 3; r1++) {
          D2[k + 3 * r1] += E_B[k + 3 * rtemp] * E_B[r1 + 3 * rtemp];
        }
      }
    }

    /* 'OLDX_MMKF:45' D2=D2/(N+1); */
    for (k = 0; k < 9; k++) {
      D2[k] /= Param[5] + 1.0;
    }

    /* 'OLDX_MMKF:46' C=C*flt+(1-flt)*abs(D2/D1*40-I); */
    r1 = 0;
    r2 = 1;
    r3 = 2;
    maxval = fabs(D1[0]);
    a21 = fabs(D1[1]);
    if (a21 > maxval) {
      maxval = a21;
      r1 = 1;
      r2 = 0;
    }

    if (fabs(D1[2]) > maxval) {
      r1 = 2;
      r2 = 1;
      r3 = 0;
    }

    D1[r2] /= D1[r1];
    D1[r3] /= D1[r1];
    D1[3 + r2] -= D1[r2] * D1[3 + r1];
    D1[3 + r3] -= D1[r3] * D1[3 + r1];
    D1[6 + r2] -= D1[r2] * D1[6 + r1];
    D1[6 + r3] -= D1[r3] * D1[6 + r1];
    if (fabs(D1[3 + r3]) > fabs(D1[3 + r2])) {
      rtemp = r2;
      r2 = r3;
      r3 = rtemp;
    }

    D1[3 + r3] /= D1[3 + r2];
    D1[6 + r3] -= D1[3 + r3] * D1[6 + r2];
    for (k = 0; k < 3; k++) {
      b_A[k + 3 * r1] = D2[k] / D1[r1];
      b_A[k + 3 * r2] = D2[3 + k] - b_A[k + 3 * r1] * D1[3 + r1];
      b_A[k + 3 * r3] = D2[6 + k] - b_A[k + 3 * r1] * D1[6 + r1];
      b_A[k + 3 * r2] /= D1[3 + r2];
      b_A[k + 3 * r3] -= b_A[k + 3 * r2] * D1[6 + r2];
      b_A[k + 3 * r3] /= D1[6 + r3];
      b_A[k + 3 * r2] -= b_A[k + 3 * r3] * D1[3 + r3];
      b_A[k + 3 * r1] -= b_A[k + 3 * r3] * D1[r3];
      b_A[k + 3 * r1] -= b_A[k + 3 * r2] * D1[r2];
    }

    for (k = 0; k < 9; k++) {
      C[k] = C[k] * Param[7] + (1.0 - Param[7]) * fabs(b_A[k] * 40.0 - (double)
        I[k]);
    }

    /* 'OLDX_MMKF:47' if(abs(C(1)-1)>dead && abs(C(1)-1)<0.98 && Param(5)) */
    if ((fabs(C[0] - 1.0) > Param[6]) && (fabs(C[0] - 1.0) < 0.98) && (Param[4]
         != 0.0)) {
      /* 'OLDX_MMKF:47' H(1)=0; */
      H[0] = 0.0;
    }

    /* 'OLDX_MMKF:48' if(abs(C(2)-1)>dead && abs(C(2)-1)<0.98 && Param(5)) */
    if ((fabs(C[1] - 1.0) > Param[6]) && (fabs(C[1] - 1.0) < 0.98) && (Param[4]
         != 0.0)) {
      /* 'OLDX_MMKF:48' H(5)=0; */
      H[4] = 0.0;
    }

    /* 'OLDX_MMKF:49' if(abs(C(3)-1)>dead && abs(C(3)-1)<0.98 && Param(5)) */
    if ((fabs(C[2] - 1.0) > Param[6]) && (fabs(C[2] - 1.0) < 0.98) && (Param[4]
         != 0.0)) {
      /* 'OLDX_MMKF:49' H(9)=0; */
      H[8] = 0.0;
    }

    /* ------adaption of R------------- */
    /* ------------------------------- */
    /* K=P_pre*H'*inv(H*P_pre*H'+R) */
    /* 'OLDX_MMKF:55' K=P_pre*H'*inv(H*P_pre*H'+R); */
    for (k = 0; k < 3; k++) {
      for (r1 = 0; r1 < 3; r1++) {
        b_A[k + 3 * r1] = 0.0;
        for (rtemp = 0; rtemp < 3; rtemp++) {
          b_A[k + 3 * r1] += H[k + 3 * rtemp] * P_pre[rtemp + 3 * r1];
        }
      }

      for (r1 = 0; r1 < 3; r1++) {
        maxval = 0.0;
        for (rtemp = 0; rtemp < 3; rtemp++) {
          maxval += b_A[k + 3 * rtemp] * H[r1 + 3 * rtemp];
        }

        d_H[k + 3 * r1] = maxval + R[k + 3 * r1];
      }
    }

    inv(d_H, D1);
    for (k = 0; k < 3; k++) {
      for (r1 = 0; r1 < 3; r1++) {
        b_A[k + 3 * r1] = 0.0;
        for (rtemp = 0; rtemp < 3; rtemp++) {
          b_A[k + 3 * r1] += P_pre[k + 3 * rtemp] * H[r1 + 3 * rtemp];
        }
      }

      for (r1 = 0; r1 < 3; r1++) {
        D2[k + 3 * r1] = 0.0;
        for (rtemp = 0; rtemp < 3; rtemp++) {
          D2[k + 3 * r1] += b_A[k + 3 * rtemp] * D1[rtemp + 3 * r1];
        }
      }
    }

    /* 'OLDX_MMKF:56' K_B(:,:,1)=K; */
    for (k = 0; k < 3; k++) {
      for (r1 = 0; r1 < 3; r1++) {
        K_B[r1 + 3 * k] = D2[r1 + 3 * k];
      }
    }

    /* 'OLDX_MMKF:57' for i=1:Param(4)+1 */
    for (rtemp = 0; rtemp < (int)(Param[3] + 1.0); rtemp++) {
      /* 'OLDX_MMKF:58' E=E+K_B(:,:,i)*E_B(:,i)/(Param(4)+1); */
      for (k = 0; k < 3; k++) {
        maxval = 0.0;
        for (r1 = 0; r1 < 3; r1++) {
          maxval += K_B[(k + 3 * r1) + 9 * rtemp] * E_B[r1 + 3 * rtemp];
        }

        E[k] += maxval / (Param[3] + 1.0);
      }
    }

    /* X=X_pre+E */
    /* 'OLDX_MMKF:61' X=X_pre+E; */
    /*  P=(I-K*H)*P_pre */
    /* 'OLDX_MMKF:63' P=(I-K*H)*P_pre; */
    for (rtemp = 0; rtemp < 3; rtemp++) {
      X[rtemp] = X_pre[rtemp] + E[rtemp];
      for (k = 0; k < 3; k++) {
        maxval = 0.0;
        for (r1 = 0; r1 < 3; r1++) {
          maxval += D2[rtemp + 3 * r1] * H[r1 + 3 * k];
        }

        b_A[rtemp + 3 * k] = (double)I[rtemp + 3 * k] - maxval;
      }

      for (k = 0; k < 3; k++) {
        P[rtemp + 3 * k] = 0.0;
        for (r1 = 0; r1 < 3; r1++) {
          P[rtemp + 3 * k] += b_A[rtemp + 3 * r1] * P_pre[r1 + 3 * k];
        }
      }
    }
  } else {
    /* 'OLDX_MMKF:64' else */
    /* 'OLDX_MMKF:65' X=X_pre; */
    for (rtemp = 0; rtemp < 3; rtemp++) {
      X[rtemp] = X_pre[rtemp];
    }

    /* 'OLDX_MMKF:66' P=P_pre; */
    memcpy(&P[0], &P_pre[0], 9U * sizeof(double));
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void OLDX_MMKF_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void OLDX_MMKF_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for OLDX_MMKF.c
 *
 * [EOF]
 */
