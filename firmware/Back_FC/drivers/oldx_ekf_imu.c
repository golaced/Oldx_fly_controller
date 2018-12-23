/*
 * File: oldx_ekf_imu.c
 *
 * MATLAB Coder version            : 3.0
 * C/C++ source code generated on  : 25-Nov-2017 09:57:40
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "oldx_ekf_imu.h"

/* Function Declarations */
static void invNxN(const double x[16], double y[16]);

/* Function Definitions */

/*
 * Arguments    : const double x[16]
 *                double y[16]
 * Return Type  : void
 */
static void invNxN(const double x[16], double y[16])
{
  double A[16];
  int i0;
  signed char ipiv[4];
  int j;
  int c;
  int jBcol;
  int ix;
  double smax;
  int k;
  double s;
  int i;
  int kAcol;
  signed char p[4];
  for (i0 = 0; i0 < 16; i0++) {
    y[i0] = 0.0;
    A[i0] = x[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    ipiv[i0] = (signed char)(1 + i0);
  }

  for (j = 0; j < 3; j++) {
    c = j * 5;
    jBcol = 0;
    ix = c;
    smax = fabs(A[c]);
    for (k = 2; k <= 4 - j; k++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        jBcol = k - 1;
        smax = s;
      }
    }

    if (A[c + jBcol] != 0.0) {
      if (jBcol != 0) {
        ipiv[j] = (signed char)((j + jBcol) + 1);
        ix = j;
        jBcol += j;
        for (k = 0; k < 4; k++) {
          smax = A[ix];
          A[ix] = A[jBcol];
          A[jBcol] = smax;
          ix += 4;
          jBcol += 4;
        }
      }

      i0 = (c - j) + 4;
      for (i = c + 1; i + 1 <= i0; i++) {
        A[i] /= A[c];
      }
    }

    jBcol = c;
    kAcol = c + 4;
    for (i = 1; i <= 3 - j; i++) {
      smax = A[kAcol];
      if (A[kAcol] != 0.0) {
        ix = c + 1;
        i0 = (jBcol - j) + 8;
        for (k = 5 + jBcol; k + 1 <= i0; k++) {
          A[k] += A[ix] * -smax;
          ix++;
        }
      }

      kAcol += 4;
      jBcol += 4;
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    p[i0] = (signed char)(1 + i0);
  }

  for (k = 0; k < 3; k++) {
    if (ipiv[k] > 1 + k) {
      jBcol = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = (signed char)jBcol;
    }
  }

  for (k = 0; k < 4; k++) {
    c = p[k] - 1;
    y[k + ((p[k] - 1) << 2)] = 1.0;
    for (j = k; j + 1 < 5; j++) {
      if (y[j + (c << 2)] != 0.0) {
        for (i = j + 1; i + 1 < 5; i++) {
          y[i + (c << 2)] -= y[j + (c << 2)] * A[i + (j << 2)];
        }
      }
    }
  }

  for (j = 0; j < 4; j++) {
    jBcol = j << 2;
    for (k = 3; k >= 0; k += -1) {
      kAcol = k << 2;
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
 * Arguments    : double X[7]
 *                double P[49]
 *                double yaw_in
 *                double T
 *                double gx
 *                double gy
 *                double gz
 *                double ax
 *                double ay
 *                double az
 *                double hx
 *                double hy
 *                double hz
 *                double n_q
 *                double n_w
 *                double n_a
 *                double n_m
 *                double Att[4]
 * Return Type  : void
 */
void oldx_ekf_imu(double X[7], double P[49], double yaw_in, double T, double gx,
                  double gy, double gz, double ax, double ay, double az, double
                  hx, double hy, double hz, double n_q, double n_w, double n_a,
                  double n_m, double Att[4])
{
  double wx;
  double wy;
  double wz;
  double reference_v_idx_0;
  double reference_v_idx_1;
  double reference_v_idx_2;
  int pn;
  double mag_sim_3d_idx_0;
  double mag_sim_3d_idx_1;
  double mag_norm;
  double yaw;
  double n_m1;
  double v[4];
  double Rk[16];
  int j;
  double b_v[7];
  double Qk[49];
  double X_pre[7];
  double Z_pre[4];
  double Ak[49];
  static const signed char iv0[7] = { 0, 0, 0, 0, 1, 0, 0 };

  static const signed char iv1[7] = { 0, 0, 0, 0, 0, 1, 0 };

  static const signed char iv2[7] = { 0, 0, 0, 0, 0, 0, 1 };

  double D1;
  double D2;
  double Hk[28];
  double b_Ak[49];
  double P_pre[49];
  int i1;
  int i2;
  double y;
  double b_Hk[28];
  double x[16];
  double b_y[16];
  double K[28];
  double b_ax[4];
  double b_Qk[49];
  double c_Qk[49];
  double b_K[49];
  double norm;
  double c_y;
  double d_y;
  double e_y;

  /* 'oldx_ekf_imu:2' en=1; */
  /* 'oldx_ekf_imu:3' en1=1; */
  /* 'oldx_ekf_imu:4' en3=1; */
  /* 'oldx_ekf_imu:5' G=9.81; */
  /* 'oldx_ekf_imu:6' wx=gx-X(5)*en; */
  wx = gx - X[4];

  /* 'oldx_ekf_imu:7' wy=gy-X(6)*en; */
  wy = gy - X[5];

  /* 'oldx_ekf_imu:8' wz=gz-X(7)*en; */
  wz = gz - X[6];

  /* 'oldx_ekf_imu:9' ax1=-ax/4096*G; */
  /* 'oldx_ekf_imu:10' ay1=-ay/4096*G; */
  /* 'oldx_ekf_imu:11' az1=-az/4096*G; */
  /* 'oldx_ekf_imu:12' reference_v=zeros(3,1); */
  /* 'oldx_ekf_imu:13' reference_v(1) = 2*(X(2)*X(4) - X(1)*X(3)); */
  reference_v_idx_0 = 2.0 * (X[1] * X[3] - X[0] * X[2]);

  /* 'oldx_ekf_imu:14' reference_v(2) = 2*(X(2)*X(2) + X(3)*X(4)); */
  reference_v_idx_1 = 2.0 * (X[1] * X[1] + X[2] * X[3]);

  /* 'oldx_ekf_imu:15' reference_v(3) = 1 - 2*(X(2)*X(2) + X(3)*X(3)); */
  reference_v_idx_2 = 1.0 - 2.0 * (X[1] * X[1] + X[2] * X[2]);

  /* 'oldx_ekf_imu:16' in=zeros(3,1); */
  /* 'oldx_ekf_imu:17' in(1)=hx; */
  /* 'oldx_ekf_imu:17' in(2)=hy; */
  /* 'oldx_ekf_imu:17' in(3)=hz; */
  /* 'oldx_ekf_imu:18' h_tmp_x = sqrt((reference_v(3))^2 + (reference_v(2))^2); */
  /* 'oldx_ekf_imu:19' h_tmp_y = sqrt((reference_v(3))^2 + (reference_v(1))^2); */
  /* 'oldx_ekf_imu:20' if(reference_v(3)<0) */
  if (reference_v_idx_2 < 0.0) {
    /* 'oldx_ekf_imu:21' pn = -1; */
    pn = -1;
  } else {
    /* 'oldx_ekf_imu:22' else */
    /* 'oldx_ekf_imu:23' pn=1; */
    pn = 1;
  }

  /* 'oldx_ekf_imu:25' mag_sim_3d=zeros(3,1); */
  /* 'oldx_ekf_imu:26' mag_sim_3d(1) = ( h_tmp_x *in(1) - pn *reference_v(1) *in(3) ) ; */
  mag_sim_3d_idx_0 = sqrt(reference_v_idx_2 * reference_v_idx_2 +
    reference_v_idx_1 * reference_v_idx_1) * hx - (double)pn * reference_v_idx_0
    * hz;

  /* 'oldx_ekf_imu:27' mag_sim_3d(2) = ( pn *h_tmp_y *in(2) - reference_v(2) *in(3) ) ; */
  mag_sim_3d_idx_1 = (double)pn * sqrt(reference_v_idx_2 * reference_v_idx_2 +
    reference_v_idx_0 * reference_v_idx_0) * hy - reference_v_idx_1 * hz;

  /* 'oldx_ekf_imu:28' mag_sim_3d(3) = reference_v(1) *in(1) + reference_v(2) *in(2) + reference_v(3) *in(3) ; */
  /* 'oldx_ekf_imu:29' mag_norm = sqrt(mag_sim_3d(1)^2+mag_sim_3d(2)^2); */
  mag_norm = sqrt(mag_sim_3d_idx_0 * mag_sim_3d_idx_0 + mag_sim_3d_idx_1 *
                  mag_sim_3d_idx_1);

  /* 'oldx_ekf_imu:30' if(yaw_in~=0) */
  if (yaw_in != 0.0) {
    /* 'oldx_ekf_imu:31' yaw=yaw_in/57.3; */
    yaw = yaw_in / 57.3;
  } else {
    /* 'oldx_ekf_imu:32' else */
    /* 'oldx_ekf_imu:33' yaw = atan( ( mag_sim_3d(1)/(mag_norm+0.0001) ) / ( mag_sim_3d(2)/(mag_norm+0.0001)) ) ; */
    yaw = atan(mag_sim_3d_idx_0 / (mag_norm + 0.0001) / (mag_sim_3d_idx_1 /
                (mag_norm + 0.0001)));
  }

  /* 'oldx_ekf_imu:35' if( mag_sim_3d(1) ~= 0 && mag_sim_3d(2) ~= 0 && mag_sim_3d(3)~= 0 && mag_norm ~= 0) */
  if ((mag_sim_3d_idx_0 != 0.0) && (mag_sim_3d_idx_1 != 0.0) &&
      ((reference_v_idx_0 * hx + reference_v_idx_1 * hy) + reference_v_idx_2 *
       hz != 0.0) && (mag_norm != 0.0)) {
    /* 'oldx_ekf_imu:36' n_m1=n_m; */
    n_m1 = n_m;
  } else {
    /* 'oldx_ekf_imu:37' else */
    /* 'oldx_ekf_imu:38' n_m1=10000; */
    n_m1 = 10000.0;
  }

  /* 'oldx_ekf_imu:42' Z=[ax1;ay1;az1;yaw]; */
  /* 'oldx_ekf_imu:42' Rk=diag([n_a;n_a;n_a;n_m1]); */
  v[0] = n_a;
  v[1] = n_a;
  v[2] = n_a;
  v[3] = n_m1;
  memset(&Rk[0], 0, sizeof(double) << 4);
  for (j = 0; j < 4; j++) {
    Rk[j + (j << 2)] = v[j];
  }

  /* Z=[ax1;ay1;az1];Rk=diag([n_a;n_a;n_a]); */
  /* 'oldx_ekf_imu:44' Qk=diag([n_q;n_q;n_q;n_q;n_w;n_w;n_w]); */
  b_v[0] = n_q;
  b_v[1] = n_q;
  b_v[2] = n_q;
  b_v[3] = n_q;
  b_v[4] = n_w;
  b_v[5] = n_w;
  b_v[6] = n_w;
  memset(&Qk[0], 0, 49U * sizeof(double));

  /* 'oldx_ekf_imu:45' X_pre=zeros(7,1); */
  /*  1 ????       */
  /* 'oldx_ekf_imu:48' X_pre(1) = X(1) + T/2*(-gx*X(2)-gy*X(3)-gz*X(4))+en3*(T/2*(X(5)*X(2)+X(6)*X(2)+X(7)*X(3))); */
  X_pre[0] = (X[0] + T / 2.0 * ((-gx * X[1] - gy * X[2]) - gz * X[3])) + T / 2.0
    * ((X[4] * X[1] + X[5] * X[1]) + X[6] * X[2]);

  /* 'oldx_ekf_imu:49' X_pre(2) = X(2) + T/2*(gx*X(1)+gz*X(3)-gy*X(4))+en3*(T/2*(-X(5)*X(1)+X(6)*X(4)-X(7)*X(3))); */
  X_pre[1] = (X[1] + T / 2.0 * ((gx * X[0] + gz * X[2]) - gy * X[3])) + T / 2.0 *
    ((-X[4] * X[0] + X[5] * X[3]) - X[6] * X[2]);

  /* 'oldx_ekf_imu:50' X_pre(3) = X(3) + T/2*(gy*X(1)-gz*X(2)+gx*X(4))+en3*(T/2*(-X(5)*X(4)-X(6)*X(1)+X(7)*X(2))); */
  X_pre[2] = (X[2] + T / 2.0 * ((gy * X[0] - gz * X[1]) + gx * X[3])) + T / 2.0 *
    ((-X[4] * X[3] - X[5] * X[0]) + X[6] * X[1]);

  /* 'oldx_ekf_imu:51' X_pre(4) = X(4) + T/2*(gz*X(1)+gy*X(2)-gx*X(3))+en3*(T/2*(X(5)*X(3)-X(6)*X(2)+X(7)*X(1))); */
  X_pre[3] = (X[3] + T / 2.0 * ((gz * X[0] + gy * X[1]) - gx * X[2])) + T / 2.0 *
    ((X[4] * X[2] - X[5] * X[1]) + X[6] * X[0]);

  /* 'oldx_ekf_imu:52' X_pre(5) = X(5); */
  X_pre[4] = X[4];

  /* 'oldx_ekf_imu:53' X_pre(6) = X(6); */
  X_pre[5] = X[5];

  /* 'oldx_ekf_imu:54' X_pre(7) = X(7); */
  X_pre[6] = X[6];

  /*  norm=sqrt(X_pre(1)^2+X_pre(2)^2+X_pre(3)^2+X_pre(4)^2); */
  /*  X_pre(1)=X_pre(1)/norm; */
  /*  X_pre(2)=X_pre(2)/norm; */
  /*  X_pre(3)=X_pre(3)/norm; */
  /*  X_pre(4)=X_pre(4)/norm; */
  /*  2  ???? */
  /* 'oldx_ekf_imu:61' Z_pre=zeros(4,1); */
  /* 'oldx_ekf_imu:62' Z_pre(1)=-2*G*(X_pre(2)*X_pre(4)-X_pre(1)*X_pre(3)); */
  Z_pre[0] = -19.62 * (X_pre[1] * X_pre[3] - X_pre[0] * X_pre[2]);

  /* 'oldx_ekf_imu:63' Z_pre(2)=-2*G*(X_pre(3)*X_pre(4)+X_pre(1)*X_pre(2)); */
  Z_pre[1] = -19.62 * (X_pre[2] * X_pre[3] + X_pre[0] * X_pre[1]);

  /* 'oldx_ekf_imu:64' Z_pre(3)=-G*(X_pre(1)^2-X_pre(2)^2-X_pre(3)^2+X_pre(4)^2); */
  Z_pre[2] = -9.81 * (((X_pre[0] * X_pre[0] - X_pre[1] * X_pre[1]) - X_pre[2] *
                       X_pre[2]) + X_pre[3] * X_pre[3]);

  /* 'oldx_ekf_imu:65' Z_pre(4)=atan(-2*(X_pre(2)*X_pre(3)-X_pre(1)*X_pre(4))/(X_pre(1)^2-X_pre(2)^2+X_pre(3)^2-X_pre(4)^2)); */
  Z_pre[3] = atan(-2.0 * (X_pre[1] * X_pre[2] - X_pre[0] * X_pre[3]) / (((X_pre
    [0] * X_pre[0] - X_pre[1] * X_pre[1]) + X_pre[2] * X_pre[2]) - X_pre[3] *
    X_pre[3]));

  /* 'oldx_ekf_imu:67' Ak =   [1,         -T*wx/2,     -T*wy/2,        -T*wz/2,        T*X(2)/2*en1,       T*X(3)/2*en1,       T*X(4)/2*en1; */
  /* 'oldx_ekf_imu:68'         T*wx/2,     1,           T*wz/2,        -T*wy/2,       -T*X(1)/2*en1,       T*X(4)/2*en1,      -T*X(3)/2*en1; */
  /* 'oldx_ekf_imu:69'         T*wy/2,    -T*wz/2,      1,              T*wx/2,       -T*X(4)/2*en1,      -T*X(1)/2*en1,       T*X(2)/2*en1; */
  /* 'oldx_ekf_imu:70'         T*wz/2,     T*wy/2,     -T*wx/2,         1,             T*X(3)/2*en1,      -T*X(2)/2*en1,      -T*X(1)/2*en1; */
  /* 'oldx_ekf_imu:71'         0,0,0,0,1,0,0; */
  /* 'oldx_ekf_imu:72'         0,0,0,0,0,1,0; */
  /* 'oldx_ekf_imu:73'         0,0,0,0,0,0,1]; */
  Ak[0] = 1.0;
  Ak[7] = -T * wx / 2.0;
  Ak[14] = -T * wy / 2.0;
  Ak[21] = -T * wz / 2.0;
  Ak[28] = T * X[1] / 2.0;
  Ak[35] = T * X[2] / 2.0;
  Ak[42] = T * X[3] / 2.0;
  Ak[1] = T * wx / 2.0;
  Ak[8] = 1.0;
  Ak[15] = T * wz / 2.0;
  Ak[22] = -T * wy / 2.0;
  Ak[29] = -T * X[0] / 2.0;
  Ak[36] = T * X[3] / 2.0;
  Ak[43] = -T * X[2] / 2.0;
  Ak[2] = T * wy / 2.0;
  Ak[9] = -T * wz / 2.0;
  Ak[16] = 1.0;
  Ak[23] = T * wx / 2.0;
  Ak[30] = -T * X[3] / 2.0;
  Ak[37] = -T * X[0] / 2.0;
  Ak[44] = T * X[1] / 2.0;
  Ak[3] = T * wz / 2.0;
  Ak[10] = T * wy / 2.0;
  Ak[17] = -T * wx / 2.0;
  Ak[24] = 1.0;
  Ak[31] = T * X[2] / 2.0;
  Ak[38] = -T * X[1] / 2.0;
  Ak[45] = -T * X[0] / 2.0;
  for (j = 0; j < 7; j++) {
    Qk[j + 7 * j] = b_v[j];
    Ak[4 + 7 * j] = iv0[j];
    Ak[5 + 7 * j] = iv1[j];
    Ak[6 + 7 * j] = iv2[j];
  }

  /* 'oldx_ekf_imu:75' D1=X(1)^2-X(2)^2+X(3)^2-X(4)^2; */
  D1 = ((X[0] * X[0] - X[1] * X[1]) + X[2] * X[2]) - X[3] * X[3];

  /* 'oldx_ekf_imu:76' D2=X(2)*X(3)-X(1)*X(4); */
  D2 = X[1] * X[2] - X[0] * X[3];

  /* 'oldx_ekf_imu:77' Hk=[2*G*X(3),       -2*G*X(4),      2*G*X(1),       -2*G*X(2),0,0,0; */
  /* 'oldx_ekf_imu:78'    -2*G*X(2),       -2*G*X(1),     -2*G*X(4),       -2*G*X(3),0,0,0; */
  /* 'oldx_ekf_imu:79'    -2*G*X(1),        2*G*X(2),      2*G*X(3),        2*G*X(4),0,0,0; */
  /* 'oldx_ekf_imu:80'    (2*X(4)*D1+4*X(1)*D2)/(D1^2+4*D2^2),(-2*X(3)*D1-4*X(2)*D2)/(D1^2+4*D2^2),(-2*X(2)*D1+4*X(3)*D2)/(D1^2+4*D2^2),(2*X(1)*D1-4*X(4)*D2)/(D1^2+4*D2^2),0,0,0; */
  /* 'oldx_ekf_imu:81'  ]; */
  Hk[0] = 19.62 * X[2];
  Hk[4] = -19.62 * X[3];
  Hk[8] = 19.62 * X[0];
  Hk[12] = -19.62 * X[1];
  Hk[16] = 0.0;
  Hk[20] = 0.0;
  Hk[24] = 0.0;
  Hk[1] = -19.62 * X[1];
  Hk[5] = -19.62 * X[0];
  Hk[9] = -19.62 * X[3];
  Hk[13] = -19.62 * X[2];
  Hk[17] = 0.0;
  Hk[21] = 0.0;
  Hk[25] = 0.0;
  Hk[2] = -19.62 * X[0];
  Hk[6] = 19.62 * X[1];
  Hk[10] = 19.62 * X[2];
  Hk[14] = 19.62 * X[3];
  Hk[18] = 0.0;
  Hk[22] = 0.0;
  Hk[26] = 0.0;
  Hk[3] = (2.0 * X[3] * D1 + 4.0 * X[0] * D2) / (D1 * D1 + 4.0 * (D2 * D2));
  Hk[7] = (-2.0 * X[2] * D1 - 4.0 * X[1] * D2) / (D1 * D1 + 4.0 * (D2 * D2));
  Hk[11] = (-2.0 * X[1] * D1 + 4.0 * X[2] * D2) / (D1 * D1 + 4.0 * (D2 * D2));
  Hk[15] = (2.0 * X[0] * D1 - 4.0 * X[3] * D2) / (D1 * D1 + 4.0 * (D2 * D2));
  Hk[19] = 0.0;
  Hk[23] = 0.0;
  Hk[27] = 0.0;

  /* X_pre=Ak*X; */
  /* 'oldx_ekf_imu:83' P_pre=Ak*P*Ak'+Qk; */
  for (i1 = 0; i1 < 7; i1++) {
    for (i2 = 0; i2 < 7; i2++) {
      b_Ak[i1 + 7 * i2] = 0.0;
      for (j = 0; j < 7; j++) {
        b_Ak[i1 + 7 * i2] += Ak[i1 + 7 * j] * P[j + 7 * i2];
      }
    }

    for (i2 = 0; i2 < 7; i2++) {
      y = 0.0;
      for (j = 0; j < 7; j++) {
        y += b_Ak[i1 + 7 * j] * Ak[i2 + 7 * j];
      }

      P_pre[i1 + 7 * i2] = y + Qk[i1 + 7 * i2];
    }
  }

  /* 'oldx_ekf_imu:84' K=P_pre*Hk'*inv((Hk*P_pre*Hk'+Rk)); */
  for (i1 = 0; i1 < 4; i1++) {
    for (i2 = 0; i2 < 7; i2++) {
      b_Hk[i1 + (i2 << 2)] = 0.0;
      for (j = 0; j < 7; j++) {
        b_Hk[i1 + (i2 << 2)] += Hk[i1 + (j << 2)] * P_pre[j + 7 * i2];
      }
    }

    for (i2 = 0; i2 < 4; i2++) {
      y = 0.0;
      for (j = 0; j < 7; j++) {
        y += b_Hk[i1 + (j << 2)] * Hk[i2 + (j << 2)];
      }

      x[i1 + (i2 << 2)] = y + Rk[i1 + (i2 << 2)];
    }
  }

  invNxN(x, b_y);
  for (i1 = 0; i1 < 7; i1++) {
    for (i2 = 0; i2 < 4; i2++) {
      b_Hk[i1 + 7 * i2] = 0.0;
      for (j = 0; j < 7; j++) {
        b_Hk[i1 + 7 * i2] += P_pre[i1 + 7 * j] * Hk[i2 + (j << 2)];
      }
    }

    for (i2 = 0; i2 < 4; i2++) {
      K[i1 + 7 * i2] = 0.0;
      for (j = 0; j < 4; j++) {
        K[i1 + 7 * i2] += b_Hk[i1 + 7 * j] * b_y[j + (i2 << 2)];
      }
    }
  }

  /* 'oldx_ekf_imu:85' X=X_pre+K*(Z-Z_pre); */
  b_ax[0] = -ax / 4096.0 * 9.81;
  b_ax[1] = -ay / 4096.0 * 9.81;
  b_ax[2] = -az / 4096.0 * 9.81;
  b_ax[3] = yaw;
  for (i1 = 0; i1 < 4; i1++) {
    v[i1] = b_ax[i1] - Z_pre[i1];
  }

  for (i1 = 0; i1 < 7; i1++) {
    y = 0.0;
    for (i2 = 0; i2 < 4; i2++) {
      y += K[i1 + 7 * i2] * v[i2];
    }

    X[i1] = X_pre[i1] + y;
  }

  /* 'oldx_ekf_imu:86' P=(eye(7)-K*Hk)*P_pre*(eye(7)-K*Hk)'+K*Rk*K'; */
  memset(&Qk[0], 0, 49U * sizeof(double));
  for (j = 0; j < 7; j++) {
    Qk[j + 7 * j] = 1.0;
  }

  memset(&Ak[0], 0, 49U * sizeof(double));
  for (j = 0; j < 7; j++) {
    Ak[j + 7 * j] = 1.0;
    for (i1 = 0; i1 < 7; i1++) {
      y = 0.0;
      for (i2 = 0; i2 < 4; i2++) {
        y += K[j + 7 * i2] * Hk[i2 + (i1 << 2)];
      }

      b_Qk[j + 7 * i1] = Qk[j + 7 * i1] - y;
    }

    for (i1 = 0; i1 < 7; i1++) {
      c_Qk[j + 7 * i1] = 0.0;
      for (i2 = 0; i2 < 7; i2++) {
        c_Qk[j + 7 * i1] += b_Qk[j + 7 * i2] * P_pre[i2 + 7 * i1];
      }
    }
  }

  for (i1 = 0; i1 < 7; i1++) {
    for (i2 = 0; i2 < 7; i2++) {
      y = 0.0;
      for (j = 0; j < 4; j++) {
        y += K[i2 + 7 * j] * Hk[j + (i1 << 2)];
      }

      b_Ak[i1 + 7 * i2] = Ak[i2 + 7 * i1] - y;
    }

    for (i2 = 0; i2 < 4; i2++) {
      b_Hk[i1 + 7 * i2] = 0.0;
      for (j = 0; j < 4; j++) {
        b_Hk[i1 + 7 * i2] += K[i1 + 7 * j] * Rk[j + (i2 << 2)];
      }
    }
  }

  for (i1 = 0; i1 < 7; i1++) {
    for (i2 = 0; i2 < 7; i2++) {
      b_Qk[i1 + 7 * i2] = 0.0;
      for (j = 0; j < 7; j++) {
        b_Qk[i1 + 7 * i2] += c_Qk[i1 + 7 * j] * b_Ak[j + 7 * i2];
      }

      b_K[i1 + 7 * i2] = 0.0;
      for (j = 0; j < 4; j++) {
        b_K[i1 + 7 * i2] += b_Hk[i1 + 7 * j] * K[i2 + 7 * j];
      }
    }
  }

  for (i1 = 0; i1 < 7; i1++) {
    for (i2 = 0; i2 < 7; i2++) {
      P[i2 + 7 * i1] = b_Qk[i2 + 7 * i1] + b_K[i2 + 7 * i1];
    }
  }

  /* 'oldx_ekf_imu:87' norm=sqrt(X(1)^2+X(2)^2+X(3)^2+X(4)^2); */
  norm = sqrt(((X[0] * X[0] + X[1] * X[1]) + X[2] * X[2]) + X[3] * X[3]);

  /* 'oldx_ekf_imu:88' X(1)=X(1)/norm; */
  y = X[0] / norm;
  X[0] /= norm;

  /* 'oldx_ekf_imu:89' X(2)=X(2)/norm; */
  c_y = X[1] / norm;
  X[1] /= norm;

  /* 'oldx_ekf_imu:90' X(3)=X(3)/norm; */
  d_y = X[2] / norm;
  X[2] /= norm;

  /* 'oldx_ekf_imu:91' X(4)=X(4)/norm; */
  e_y = X[3] / norm;
  X[3] /= norm;

  /* 'oldx_ekf_imu:92' Att=zeros(4,1); */
  /* 'oldx_ekf_imu:93' Att(2) = atan(2*(X(1)*X(2) + X(3)*X(4))/(1 - 2*(X(2)*X(2) + X(3)*X(3))))*57.3; */
  Att[1] = atan(2.0 * (y * c_y + d_y * e_y) / (1.0 - 2.0 * (c_y * c_y + d_y *
    d_y))) * 57.3;

  /* 'oldx_ekf_imu:94' Att(1) = asin(2*(X(2)*X(4) - X(1)*X(3))) *57.3; */
  Att[0] = asin(2.0 * (c_y * e_y - y * d_y)) * 57.3;

  /* 'oldx_ekf_imu:95' Att(3) = atan(2*(-X(2)*X(3) - X(1)*X(4))/(2*(X(1)*X(1) + X(2)*X(2)) - 1))*57.3; */
  Att[2] = atan(2.0 * (-c_y * d_y - y * e_y) / (2.0 * (y * y + c_y * c_y) - 1.0))
    * 57.3;

  /* 'oldx_ekf_imu:96' Att(4) = yaw; */
  Att[3] = yaw;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void oldx_ekf_imu_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void oldx_ekf_imu_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for oldx_ekf_imu.c
 *
 * [EOF]
 */
