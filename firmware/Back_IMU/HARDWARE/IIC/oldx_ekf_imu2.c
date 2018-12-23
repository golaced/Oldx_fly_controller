/*
 * File: oldx_ekf_imu2.c
 *
 * MATLAB Coder version            : 3.0
 * C/C++ source code generated on  : 28-Nov-2017 21:48:26
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "oldx_ekf_imu2.h"

/* Variable Definitions */
static boolean_T Ji_not_empty;
static float Q[144];
static boolean_T Q_not_empty;

/* Function Declarations */
static void diag(const float v[12], float d[144]);
static void mpower(const float a[9], float c[9]);
static float norm(const float x[3]);
static float rt_atan2f_snf(float u0, float u1);

/* Function Definitions */

/*
 * Arguments    : const float v[12]
 *                float d[144]
 * Return Type  : void
 */
static void diag(const float v[12], float d[144])
{
  int j;
  memset(&d[0], 0, 144U * sizeof(float));
  for (j = 0; j < 12; j++) {
    d[j + 12 * j] = v[j];
  }
}

/*
 * Arguments    : const float a[9]
 *                float c[9]
 * Return Type  : void
 */
static void mpower(const float a[9], float c[9])
{
  int i0;
  int i1;
  int i2;
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      c[i0 + 3 * i1] = 0.0F;
      for (i2 = 0; i2 < 3; i2++) {
        c[i0 + 3 * i1] += a[i0 + 3 * i2] * a[i2 + 3 * i1];
      }
    }
  }
}

/*
 * Arguments    : const float x[3]
 * Return Type  : float
 */
static float norm(const float x[3])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 3; k++) {
    absxk = (float)fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (float)sqrt(y);
}

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_atan2f_snf(float u0, float u1)
{
  float y;
  int b_u0;
  int b_u1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0F) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = (float)atan2((float)b_u0, (float)b_u1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (float)atan2(u0, u1);
  }

  return y;
}

/*
 * Arguments    : float x_apo[12]
 *                float P_apo[144]
 *                const float zFlag[3]
 *                const float z[9]
 *                const float param[7]
 *                float dt
 *                float xa_apo[12]
 *                float Pa_apo[144]
 *                float Rot_matrix[9]
 *                float eulerAngles[3]
 * Return Type  : void
 */
void oldx_ekf_imu2(float x_apo[12], float P_apo[144], const float zFlag[3],
                   const float z[9], const float param[7], float dt, float
                   xa_apo[12], float Pa_apo[144], float Rot_matrix[9], float
                   eulerAngles[3])
{
  float wak[3];
  float O[9];
  float temp;
  float b[9];
  float s;
  float b_b[9];
  float b_x_apo[3];
  float c_x_apo[3];
  float fv0[9];
  float fv1[9];
  int k;
  int jAcol;
  static const signed char iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  float d_x_apo[3];
  float fv2[3];
  float fv3[3];
  float x_apr[12];
  float A_lin[144];
  static const signed char iv1[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  float b_A_lin[144];
  static const signed char iv2[144] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
  };

  float b_param[12];
  float P_apr[144];
  int jp;
  float a[108];
  static const signed char b_a[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  float S_k[81];
  static const signed char c_b[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  float c_param[9];
  signed char ipiv[9];
  int j;
  int c;
  int ix;
  int jy;
  int kBcol;
  float K_k[108];
  float c_a[36];
  static const signed char d_a[36] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char d_b[36] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  float d_param[3];
  float b_c[36];
  float b_K_k[36];
  float e_a[72];
  static const signed char f_a[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0 };

  float b_S_k[36];
  static const signed char e_b[72] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0 };

  float e_param[6];
  signed char b_ipiv[6];
  float c_K_k[72];
  static const signed char g_a[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1 };

  static const signed char f_b[72] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1 };

  float h_a[6];
  float b_z[6];
  float z_n_b[3];
  float y_n_b[3];

  /* LQG Position Estimator and Controller */
  /*  Observer: */
  /*         x[n|n]   = x[n|n-1] + M(y[n] - Cx[n|n-1] - Du[n]) */
  /*         x[n+1|n] = Ax[n|n] + Bu[n] */
  /*  */
  /*  $Author: Tobias Naegeli $    $Date: 2014 $    $Revision: 3 $ */
  /*  */
  /*  */
  /*  Arguments: */
  /*  approx_prediction: if 1 then the exponential map is approximated with a */
  /*  first order taylor approximation. has at the moment not a big influence */
  /*  (just 1st or 2nd order approximation) we should change it to rodriquez */
  /*  approximation. */
  /*  use_inertia_matrix: set to true if you have the inertia matrix J for your */
  /*  quadrotor */
  /*  xa_apo_k: old state vectotr */
  /*  zFlag: if sensor measurement is available [gyro, acc, mag] */
  /*  dt: dt in s */
  /*  z: measurements [gyro, acc, mag] */
  /*  q_rotSpeed: process noise gyro */
  /*  q_rotAcc: process noise gyro acceleration */
  /*  q_acc: process noise acceleration */
  /*  q_mag: process noise magnetometer */
  /*  r_gyro: measurement noise gyro */
  /*  r_accel: measurement noise accel */
  /*  r_mag: measurement noise mag */
  /*  J: moment of inertia matrix */
  /*  Output: */
  /*  xa_apo: updated state vectotr */
  /*  Pa_apo: updated state covariance matrix */
  /*  Rot_matrix: rotation matrix */
  /*  eulerAngles: euler angles */
  /*  debugOutput: not used */
  /* % model specific parameters */
  /*  compute once the inverse of the Inertia */
  if (!Ji_not_empty) {
    Ji_not_empty = true;
  }

  /* % init */
  /*  persistent x_apo */
  /*  if(isempty(x_apo)) */
  /*      gyro_init=single([0;0;0]); */
  /*      gyro_acc_init=single([0;0;0]); */
  /*      acc_init=single([0;0;-9.81]); */
  /*      mag_init=single([1;0;0]); */
  /*      x_apo=single([gyro_init;gyro_acc_init;acc_init;mag_init]); */
  /*       */
  /*  end */
  /*   */
  /*  persistent P_apo */
  /*  if(isempty(P_apo)) */
  /*      %     P_apo = single(eye(NSTATES) * 1000); */
  /*      P_apo = single(200*ones(12)); */
  /*  end */
  /* % copy the states */
  /*  x  body angular rate */
  /*  y  body angular rate */
  /*  z  body angular rate */
  /*  x  body angular acceleration */
  /*  y  body angular acceleration */
  /*  z  body angular acceleration */
  /*  x  component gravity vector */
  /*  y  component gravity vector */
  /*  z  component gravity vector */
  /*  x  component magnetic field vector */
  /*  y  component magnetic field vector */
  /*  z  component magnetic field vector */
  /* % prediction section */
  /*  compute the apriori state estimate from the previous aposteriori estimate */
  /* body angular accelerations */
  wak[0] = x_apo[3];
  wak[1] = x_apo[4];
  wak[2] = x_apo[5];

  /* body angular rates */
  /* derivative of the prediction rotation matrix */
  O[0] = 0.0F;
  O[1] = -x_apo[2];
  O[2] = x_apo[1];
  O[3] = x_apo[2];
  O[4] = 0.0F;
  O[5] = -x_apo[0];
  O[6] = -x_apo[1];
  O[7] = x_apo[0];
  O[8] = 0.0F;

  /* prediction of the earth z vector */
  temp = dt * dt / 2.0F;
  mpower(O, b);

  /* zek =expm2(O*dt)*[zex;zey;zez]; not working because use double */
  /* precision */
  /* prediction of the magnetic vector */
  s = dt * dt / 2.0F;
  mpower(O, b_b);

  /* muk =expm2(O*dt)*[mux;muy;muz]; not working because use double */
  /* precision */
  b_x_apo[0] = x_apo[0];
  b_x_apo[1] = x_apo[1];
  b_x_apo[2] = x_apo[2];
  c_x_apo[0] = x_apo[6];
  c_x_apo[1] = x_apo[7];
  c_x_apo[2] = x_apo[8];
  for (k = 0; k < 3; k++) {
    for (jAcol = 0; jAcol < 3; jAcol++) {
      fv1[jAcol + 3 * k] = ((float)iv0[jAcol + 3 * k] + O[jAcol + 3 * k] * dt) +
        temp * b[jAcol + 3 * k];
      fv0[jAcol + 3 * k] = ((float)iv0[jAcol + 3 * k] + O[jAcol + 3 * k] * dt) +
        s * b_b[jAcol + 3 * k];
    }
  }

  d_x_apo[0] = x_apo[9];
  d_x_apo[1] = x_apo[10];
  d_x_apo[2] = x_apo[11];
  for (k = 0; k < 3; k++) {
    fv2[k] = 0.0F;
    fv3[k] = 0.0F;
    for (jAcol = 0; jAcol < 3; jAcol++) {
      fv2[k] += fv1[k + 3 * jAcol] * c_x_apo[jAcol];
      fv3[k] += fv0[k + 3 * jAcol] * d_x_apo[jAcol];
    }

    x_apr[k] = b_x_apo[k] + dt * wak[k];
    x_apr[k + 3] = wak[k];
    x_apr[k + 6] = fv2[k];
    x_apr[k + 9] = fv3[k];
  }

  /*  compute the apriori error covariance estimate from the previous */
  /* aposteriori estimate */
  for (k = 0; k < 12; k++) {
    for (jAcol = 0; jAcol < 3; jAcol++) {
      A_lin[jAcol + 12 * k] = iv1[jAcol + 3 * k];
      A_lin[(jAcol + 12 * k) + 3] = 0.0F;
    }
  }

  A_lin[6] = 0.0F;
  A_lin[7] = x_apo[8];
  A_lin[8] = -x_apo[7];
  A_lin[18] = -x_apo[8];
  A_lin[19] = 0.0F;
  A_lin[20] = x_apo[6];
  A_lin[30] = x_apo[7];
  A_lin[31] = -x_apo[6];
  A_lin[32] = 0.0F;
  A_lin[9] = 0.0F;
  A_lin[10] = x_apo[11];
  A_lin[11] = -x_apo[10];
  A_lin[21] = -x_apo[11];
  A_lin[22] = 0.0F;
  A_lin[23] = x_apo[9];
  A_lin[33] = x_apo[10];
  A_lin[34] = -x_apo[9];
  A_lin[35] = 0.0F;
  for (k = 0; k < 3; k++) {
    for (jAcol = 0; jAcol < 3; jAcol++) {
      A_lin[(jAcol + 12 * (k + 3)) + 6] = 0.0F;
      A_lin[(jAcol + 12 * (k + 6)) + 6] = O[jAcol + 3 * k];
      A_lin[(jAcol + 12 * (k + 9)) + 6] = 0.0F;
      A_lin[(jAcol + 12 * (k + 3)) + 9] = 0.0F;
      A_lin[(jAcol + 12 * (k + 6)) + 9] = 0.0F;
      A_lin[(jAcol + 12 * (k + 9)) + 9] = O[jAcol + 3 * k];
    }
  }

  for (k = 0; k < 12; k++) {
    for (jAcol = 0; jAcol < 12; jAcol++) {
      b_A_lin[jAcol + 12 * k] = (float)iv2[jAcol + 12 * k] + A_lin[jAcol + 12 *
        k] * dt;
    }
  }

  /* process covariance matrix */
  if (!Q_not_empty||1) {
    b_param[0] = param[0];
    b_param[1] = param[0];
    b_param[2] = param[0];
    b_param[3] = param[1];
    b_param[4] = param[1];
    b_param[5] = param[1];
    b_param[6] = param[2];
    b_param[7] = param[2];
    b_param[8] = param[2];
    b_param[9] = param[3];
    b_param[10] = param[3];
    b_param[11] = param[3];
    diag(b_param, Q);
    Q_not_empty = true;
  }

  for (k = 0; k < 12; k++) {
    for (jAcol = 0; jAcol < 12; jAcol++) {
      A_lin[k + 12 * jAcol] = 0.0F;
      for (jp = 0; jp < 12; jp++) {
        A_lin[k + 12 * jAcol] += b_A_lin[k + 12 * jp] * P_apo[jp + 12 * jAcol];
      }
    }

    for (jAcol = 0; jAcol < 12; jAcol++) {
      temp = 0.0F;
      for (jp = 0; jp < 12; jp++) {
        temp += A_lin[k + 12 * jp] * b_A_lin[jAcol + 12 * jp];
      }

      P_apr[k + 12 * jAcol] = temp + Q[k + 12 * jAcol];
    }
  }

  /* % update */
  if ((zFlag[0] == 1.0F) && (zFlag[1] == 1.0F) && (zFlag[2] == 1.0F)) {
    /*      R=[r_gyro,0,0,0,0,0,0,0,0; */
    /*          0,r_gyro,0,0,0,0,0,0,0; */
    /*          0,0,r_gyro,0,0,0,0,0,0; */
    /*          0,0,0,r_accel,0,0,0,0,0; */
    /*          0,0,0,0,r_accel,0,0,0,0; */
    /*          0,0,0,0,0,r_accel,0,0,0; */
    /*          0,0,0,0,0,0,r_mag,0,0; */
    /*          0,0,0,0,0,0,0,r_mag,0; */
    /*          0,0,0,0,0,0,0,0,r_mag]; */
    /* observation matrix */
    /* [zw;ze;zmk]; */
    /* S_k=H_k*P_apr*H_k'+R; */
    for (k = 0; k < 9; k++) {
      for (jAcol = 0; jAcol < 12; jAcol++) {
        a[k + 9 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          a[k + 9 * jAcol] += (float)b_a[k + 9 * jp] * P_apr[jp + 12 * jAcol];
        }
      }

      for (jAcol = 0; jAcol < 9; jAcol++) {
        S_k[k + 9 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          S_k[k + 9 * jAcol] += a[k + 9 * jp] * (float)c_b[jp + 12 * jAcol];
        }
      }
    }

    c_param[0] = param[4];
    c_param[1] = param[4];
    c_param[2] = param[4];
    c_param[3] = param[5];
    c_param[4] = param[5];
    c_param[5] = param[5];
    c_param[6] = param[6];
    c_param[7] = param[6];
    c_param[8] = param[6];
    for (k = 0; k < 9; k++) {
      S_k[10 * k] += c_param[k];
      ipiv[k] = (signed char)(1 + k);
    }

    for (j = 0; j < 8; j++) {
      c = j * 10;
      jp = 0;
      ix = c;
      temp = (float)fabs(S_k[c]);
      for (k = 2; k <= 9 - j; k++) {
        ix++;
        s = (float)fabs(S_k[ix]);
        if (s > temp) {
          jp = k - 1;
          temp = s;
        }
      }

      if (S_k[c + jp] != 0.0F) {
        if (jp != 0) {
          ipiv[j] = (signed char)((j + jp) + 1);
          ix = j;
          jp += j;
          for (k = 0; k < 9; k++) {
            temp = S_k[ix];
            S_k[ix] = S_k[jp];
            S_k[jp] = temp;
            ix += 9;
            jp += 9;
          }
        }

        k = (c - j) + 9;
        for (jy = c + 1; jy + 1 <= k; jy++) {
          S_k[jy] /= S_k[c];
        }
      }

      jp = c;
      jy = c + 9;
      for (jAcol = 1; jAcol <= 8 - j; jAcol++) {
        temp = S_k[jy];
        if (S_k[jy] != 0.0F) {
          ix = c + 1;
          k = (jp - j) + 18;
          for (kBcol = 10 + jp; kBcol + 1 <= k; kBcol++) {
            S_k[kBcol] += S_k[ix] * -temp;
            ix++;
          }
        }

        jy += 9;
        jp += 9;
      }
    }

    for (k = 0; k < 12; k++) {
      for (jAcol = 0; jAcol < 9; jAcol++) {
        K_k[k + 12 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          K_k[k + 12 * jAcol] += P_apr[k + 12 * jp] * (float)c_b[jp + 12 * jAcol];
        }
      }
    }

    for (j = 0; j < 9; j++) {
      jp = 12 * j;
      jAcol = 9 * j;
      for (k = 1; k <= j; k++) {
        kBcol = 12 * (k - 1);
        if (S_k[(k + jAcol) - 1] != 0.0F) {
          for (jy = 0; jy < 12; jy++) {
            K_k[jy + jp] -= S_k[(k + jAcol) - 1] * K_k[jy + kBcol];
          }
        }
      }

      temp = 1.0F / S_k[j + jAcol];
      for (jy = 0; jy < 12; jy++) {
        K_k[jy + jp] *= temp;
      }
    }

    for (j = 8; j >= 0; j += -1) {
      jp = 12 * j;
      jAcol = 9 * j - 1;
      for (k = j + 2; k < 10; k++) {
        kBcol = 12 * (k - 1);
        if (S_k[k + jAcol] != 0.0F) {
          for (jy = 0; jy < 12; jy++) {
            K_k[jy + jp] -= S_k[k + jAcol] * K_k[jy + kBcol];
          }
        }
      }
    }

    for (jAcol = 7; jAcol >= 0; jAcol += -1) {
      if (ipiv[jAcol] != jAcol + 1) {
        jp = ipiv[jAcol] - 1;
        for (kBcol = 0; kBcol < 12; kBcol++) {
          temp = K_k[kBcol + 12 * jAcol];
          K_k[kBcol + 12 * jAcol] = K_k[kBcol + 12 * jp];
          K_k[kBcol + 12 * jp] = temp;
        }
      }
    }

    for (k = 0; k < 9; k++) {
      temp = 0.0F;
      for (jAcol = 0; jAcol < 12; jAcol++) {
        temp += (float)b_a[k + 9 * jAcol] * x_apr[jAcol];
      }

      b[k] = z[k] - temp;
    }

    for (k = 0; k < 12; k++) {
      temp = 0.0F;
      for (jAcol = 0; jAcol < 9; jAcol++) {
        temp += K_k[k + 12 * jAcol] * b[jAcol];
      }

      x_apo[k] = x_apr[k] + temp;
      for (jAcol = 0; jAcol < 12; jAcol++) {
        temp = 0.0F;
        for (jp = 0; jp < 9; jp++) {
          temp += K_k[k + 12 * jp] * (float)b_a[jp + 9 * jAcol];
        }

        A_lin[k + 12 * jAcol] = (float)iv2[k + 12 * jAcol] - temp;
      }

      for (jAcol = 0; jAcol < 12; jAcol++) {
        P_apo[k + 12 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          P_apo[k + 12 * jAcol] += A_lin[k + 12 * jp] * P_apr[jp + 12 * jAcol];
        }
      }
    }
  } else if ((zFlag[0] == 1.0F) && (zFlag[1] == 0.0F) && (zFlag[2] == 0.0F)) {
    /* observation matrix */
    /*  S_k=H_k(1:3,1:12)*P_apr*H_k(1:3,1:12)'+R(1:3,1:3); */
    for (k = 0; k < 3; k++) {
      for (jAcol = 0; jAcol < 12; jAcol++) {
        c_a[k + 3 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          c_a[k + 3 * jAcol] += (float)d_a[k + 3 * jp] * P_apr[jp + 12 * jAcol];
        }
      }

      for (jAcol = 0; jAcol < 3; jAcol++) {
        O[k + 3 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          O[k + 3 * jAcol] += c_a[k + 3 * jp] * (float)d_b[jp + 12 * jAcol];
        }
      }
    }

    d_param[0] = param[4];
    d_param[1] = param[4];
    d_param[2] = param[4];
    for (k = 0; k < 3; k++) {
      O[k << 2] += d_param[k];
    }

    for (k = 0; k < 12; k++) {
      for (jAcol = 0; jAcol < 3; jAcol++) {
        b_c[k + 12 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          b_c[k + 12 * jAcol] += P_apr[k + 12 * jp] * (float)d_b[jp + 12 * jAcol];
        }
      }
    }

    jp = 0;
    kBcol = 1;
    jy = 2;
    temp = (float)fabs(O[0]);
    s = (float)fabs(O[1]);
    if (s > temp) {
      temp = s;
      jp = 1;
      kBcol = 0;
    }

    if ((float)fabs(O[2]) > temp) {
      jp = 2;
      kBcol = 1;
      jy = 0;
    }

    O[kBcol] /= O[jp];
    O[jy] /= O[jp];
    O[3 + kBcol] -= O[kBcol] * O[3 + jp];
    O[3 + jy] -= O[jy] * O[3 + jp];
    O[6 + kBcol] -= O[kBcol] * O[6 + jp];
    O[6 + jy] -= O[jy] * O[6 + jp];
    if ((float)fabs(O[3 + jy]) > (float)fabs(O[3 + kBcol])) {
      jAcol = kBcol;
      kBcol = jy;
      jy = jAcol;
    }

    O[3 + jy] /= O[3 + kBcol];
    O[6 + jy] -= O[3 + jy] * O[6 + kBcol];
    for (k = 0; k < 12; k++) {
      b_K_k[k + 12 * jp] = b_c[k] / O[jp];
      b_K_k[k + 12 * kBcol] = b_c[12 + k] - b_K_k[k + 12 * jp] * O[3 + jp];
      b_K_k[k + 12 * jy] = b_c[24 + k] - b_K_k[k + 12 * jp] * O[6 + jp];
      b_K_k[k + 12 * kBcol] /= O[3 + kBcol];
      b_K_k[k + 12 * jy] -= b_K_k[k + 12 * kBcol] * O[6 + kBcol];
      b_K_k[k + 12 * jy] /= O[6 + jy];
      b_K_k[k + 12 * kBcol] -= b_K_k[k + 12 * jy] * O[3 + jy];
      b_K_k[k + 12 * jp] -= b_K_k[k + 12 * jy] * O[jy];
      b_K_k[k + 12 * jp] -= b_K_k[k + 12 * kBcol] * O[kBcol];
    }

    for (k = 0; k < 3; k++) {
      temp = 0.0F;
      for (jAcol = 0; jAcol < 12; jAcol++) {
        temp += (float)d_a[k + 3 * jAcol] * x_apr[jAcol];
      }

      b_x_apo[k] = z[k] - temp;
    }

    for (k = 0; k < 12; k++) {
      temp = 0.0F;
      for (jAcol = 0; jAcol < 3; jAcol++) {
        temp += b_K_k[k + 12 * jAcol] * b_x_apo[jAcol];
      }

      x_apo[k] = x_apr[k] + temp;
      for (jAcol = 0; jAcol < 12; jAcol++) {
        temp = 0.0F;
        for (jp = 0; jp < 3; jp++) {
          temp += b_K_k[k + 12 * jp] * (float)d_a[jp + 3 * jAcol];
        }

        A_lin[k + 12 * jAcol] = (float)iv2[k + 12 * jAcol] - temp;
      }

      for (jAcol = 0; jAcol < 12; jAcol++) {
        P_apo[k + 12 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          P_apo[k + 12 * jAcol] += A_lin[k + 12 * jp] * P_apr[jp + 12 * jAcol];
        }
      }
    }
  } else if ((zFlag[0] == 1.0F) && (zFlag[1] == 1.0F) && (zFlag[2] == 0.0F)) {
    /*              R=[r_gyro,0,0,0,0,0; */
    /*                  0,r_gyro,0,0,0,0; */
    /*                  0,0,r_gyro,0,0,0; */
    /*                  0,0,0,r_accel,0,0; */
    /*                  0,0,0,0,r_accel,0; */
    /*                  0,0,0,0,0,r_accel]; */
    /* observation matrix */
    /*  S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)'+R(1:6,1:6); */
    for (k = 0; k < 6; k++) {
      for (jAcol = 0; jAcol < 12; jAcol++) {
        e_a[k + 6 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          e_a[k + 6 * jAcol] += (float)f_a[k + 6 * jp] * P_apr[jp + 12 * jAcol];
        }
      }

      for (jAcol = 0; jAcol < 6; jAcol++) {
        b_S_k[k + 6 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          b_S_k[k + 6 * jAcol] += e_a[k + 6 * jp] * (float)e_b[jp + 12 * jAcol];
        }
      }
    }

    e_param[0] = param[4];
    e_param[1] = param[4];
    e_param[2] = param[4];
    e_param[3] = param[5];
    e_param[4] = param[5];
    e_param[5] = param[5];
    for (k = 0; k < 6; k++) {
      b_S_k[7 * k] += e_param[k];
      b_ipiv[k] = (signed char)(1 + k);
    }

    for (j = 0; j < 5; j++) {
      c = j * 7;
      jp = 0;
      ix = c;
      temp = (float)fabs(b_S_k[c]);
      for (k = 2; k <= 6 - j; k++) {
        ix++;
        s = (float)fabs(b_S_k[ix]);
        if (s > temp) {
          jp = k - 1;
          temp = s;
        }
      }

      if (b_S_k[c + jp] != 0.0F) {
        if (jp != 0) {
          b_ipiv[j] = (signed char)((j + jp) + 1);
          ix = j;
          jp += j;
          for (k = 0; k < 6; k++) {
            temp = b_S_k[ix];
            b_S_k[ix] = b_S_k[jp];
            b_S_k[jp] = temp;
            ix += 6;
            jp += 6;
          }
        }

        k = (c - j) + 6;
        for (jy = c + 1; jy + 1 <= k; jy++) {
          b_S_k[jy] /= b_S_k[c];
        }
      }

      jp = c;
      jy = c + 6;
      for (jAcol = 1; jAcol <= 5 - j; jAcol++) {
        temp = b_S_k[jy];
        if (b_S_k[jy] != 0.0F) {
          ix = c + 1;
          k = (jp - j) + 12;
          for (kBcol = 7 + jp; kBcol + 1 <= k; kBcol++) {
            b_S_k[kBcol] += b_S_k[ix] * -temp;
            ix++;
          }
        }

        jy += 6;
        jp += 6;
      }
    }

    for (k = 0; k < 12; k++) {
      for (jAcol = 0; jAcol < 6; jAcol++) {
        c_K_k[k + 12 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          c_K_k[k + 12 * jAcol] += P_apr[k + 12 * jp] * (float)e_b[jp + 12 *
            jAcol];
        }
      }
    }

    for (j = 0; j < 6; j++) {
      jp = 12 * j;
      jAcol = 6 * j;
      for (k = 1; k <= j; k++) {
        kBcol = 12 * (k - 1);
        if (b_S_k[(k + jAcol) - 1] != 0.0F) {
          for (jy = 0; jy < 12; jy++) {
            c_K_k[jy + jp] -= b_S_k[(k + jAcol) - 1] * c_K_k[jy + kBcol];
          }
        }
      }

      temp = 1.0F / b_S_k[j + jAcol];
      for (jy = 0; jy < 12; jy++) {
        c_K_k[jy + jp] *= temp;
      }
    }

    for (j = 5; j >= 0; j += -1) {
      jp = 12 * j;
      jAcol = 6 * j - 1;
      for (k = j + 2; k < 7; k++) {
        kBcol = 12 * (k - 1);
        if (b_S_k[k + jAcol] != 0.0F) {
          for (jy = 0; jy < 12; jy++) {
            c_K_k[jy + jp] -= b_S_k[k + jAcol] * c_K_k[jy + kBcol];
          }
        }
      }
    }

    for (jAcol = 4; jAcol >= 0; jAcol += -1) {
      if (b_ipiv[jAcol] != jAcol + 1) {
        jp = b_ipiv[jAcol] - 1;
        for (kBcol = 0; kBcol < 12; kBcol++) {
          temp = c_K_k[kBcol + 12 * jAcol];
          c_K_k[kBcol + 12 * jAcol] = c_K_k[kBcol + 12 * jp];
          c_K_k[kBcol + 12 * jp] = temp;
        }
      }
    }

    for (k = 0; k < 6; k++) {
      temp = 0.0F;
      for (jAcol = 0; jAcol < 12; jAcol++) {
        temp += (float)f_a[k + 6 * jAcol] * x_apr[jAcol];
      }

      e_param[k] = z[k] - temp;
    }

    for (k = 0; k < 12; k++) {
      temp = 0.0F;
      for (jAcol = 0; jAcol < 6; jAcol++) {
        temp += c_K_k[k + 12 * jAcol] * e_param[jAcol];
      }

      x_apo[k] = x_apr[k] + temp;
      for (jAcol = 0; jAcol < 12; jAcol++) {
        temp = 0.0F;
        for (jp = 0; jp < 6; jp++) {
          temp += c_K_k[k + 12 * jp] * (float)f_a[jp + 6 * jAcol];
        }

        A_lin[k + 12 * jAcol] = (float)iv2[k + 12 * jAcol] - temp;
      }

      for (jAcol = 0; jAcol < 12; jAcol++) {
        P_apo[k + 12 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          P_apo[k + 12 * jAcol] += A_lin[k + 12 * jp] * P_apr[jp + 12 * jAcol];
        }
      }
    }
  } else if ((zFlag[0] == 1.0F) && (zFlag[1] == 0.0F) && (zFlag[2] == 1.0F)) {
    /*                  R=[r_gyro,0,0,0,0,0; */
    /*                      0,r_gyro,0,0,0,0; */
    /*                      0,0,r_gyro,0,0,0; */
    /*                      0,0,0,r_mag,0,0; */
    /*                      0,0,0,0,r_mag,0; */
    /*                      0,0,0,0,0,r_mag]; */
    /* observation matrix */
    /* S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)'+R(1:6,1:6); */
    for (k = 0; k < 6; k++) {
      for (jAcol = 0; jAcol < 12; jAcol++) {
        e_a[k + 6 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          e_a[k + 6 * jAcol] += (float)g_a[k + 6 * jp] * P_apr[jp + 12 * jAcol];
        }
      }

      for (jAcol = 0; jAcol < 6; jAcol++) {
        b_S_k[k + 6 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          b_S_k[k + 6 * jAcol] += e_a[k + 6 * jp] * (float)f_b[jp + 12 * jAcol];
        }
      }
    }

    e_param[0] = param[4];
    e_param[1] = param[4];
    e_param[2] = param[4];
    e_param[3] = param[6];
    e_param[4] = param[6];
    e_param[5] = param[6];
    for (k = 0; k < 6; k++) {
      b_S_k[7 * k] += e_param[k];
      b_ipiv[k] = (signed char)(1 + k);
    }

    for (j = 0; j < 5; j++) {
      c = j * 7;
      jp = 0;
      ix = c;
      temp = (float)fabs(b_S_k[c]);
      for (k = 2; k <= 6 - j; k++) {
        ix++;
        s = (float)fabs(b_S_k[ix]);
        if (s > temp) {
          jp = k - 1;
          temp = s;
        }
      }

      if (b_S_k[c + jp] != 0.0F) {
        if (jp != 0) {
          b_ipiv[j] = (signed char)((j + jp) + 1);
          ix = j;
          jp += j;
          for (k = 0; k < 6; k++) {
            temp = b_S_k[ix];
            b_S_k[ix] = b_S_k[jp];
            b_S_k[jp] = temp;
            ix += 6;
            jp += 6;
          }
        }

        k = (c - j) + 6;
        for (jy = c + 1; jy + 1 <= k; jy++) {
          b_S_k[jy] /= b_S_k[c];
        }
      }

      jp = c;
      jy = c + 6;
      for (jAcol = 1; jAcol <= 5 - j; jAcol++) {
        temp = b_S_k[jy];
        if (b_S_k[jy] != 0.0F) {
          ix = c + 1;
          k = (jp - j) + 12;
          for (kBcol = 7 + jp; kBcol + 1 <= k; kBcol++) {
            b_S_k[kBcol] += b_S_k[ix] * -temp;
            ix++;
          }
        }

        jy += 6;
        jp += 6;
      }
    }

    for (k = 0; k < 12; k++) {
      for (jAcol = 0; jAcol < 6; jAcol++) {
        c_K_k[k + 12 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          c_K_k[k + 12 * jAcol] += P_apr[k + 12 * jp] * (float)f_b[jp + 12 *
            jAcol];
        }
      }
    }

    for (j = 0; j < 6; j++) {
      jp = 12 * j;
      jAcol = 6 * j;
      for (k = 1; k <= j; k++) {
        kBcol = 12 * (k - 1);
        if (b_S_k[(k + jAcol) - 1] != 0.0F) {
          for (jy = 0; jy < 12; jy++) {
            c_K_k[jy + jp] -= b_S_k[(k + jAcol) - 1] * c_K_k[jy + kBcol];
          }
        }
      }

      temp = 1.0F / b_S_k[j + jAcol];
      for (jy = 0; jy < 12; jy++) {
        c_K_k[jy + jp] *= temp;
      }
    }

    for (j = 5; j >= 0; j += -1) {
      jp = 12 * j;
      jAcol = 6 * j - 1;
      for (k = j + 2; k < 7; k++) {
        kBcol = 12 * (k - 1);
        if (b_S_k[k + jAcol] != 0.0F) {
          for (jy = 0; jy < 12; jy++) {
            c_K_k[jy + jp] -= b_S_k[k + jAcol] * c_K_k[jy + kBcol];
          }
        }
      }
    }

    for (jAcol = 4; jAcol >= 0; jAcol += -1) {
      if (b_ipiv[jAcol] != jAcol + 1) {
        jp = b_ipiv[jAcol] - 1;
        for (kBcol = 0; kBcol < 12; kBcol++) {
          temp = c_K_k[kBcol + 12 * jAcol];
          c_K_k[kBcol + 12 * jAcol] = c_K_k[kBcol + 12 * jp];
          c_K_k[kBcol + 12 * jp] = temp;
        }
      }
    }

    for (k = 0; k < 3; k++) {
      e_param[k] = z[k];
      e_param[k + 3] = z[6 + k];
    }

    for (k = 0; k < 6; k++) {
      h_a[k] = 0.0F;
      for (jAcol = 0; jAcol < 12; jAcol++) {
        h_a[k] += (float)g_a[k + 6 * jAcol] * x_apr[jAcol];
      }

      b_z[k] = e_param[k] - h_a[k];
    }

    for (k = 0; k < 12; k++) {
      temp = 0.0F;
      for (jAcol = 0; jAcol < 6; jAcol++) {
        temp += c_K_k[k + 12 * jAcol] * b_z[jAcol];
      }

      x_apo[k] = x_apr[k] + temp;
      for (jAcol = 0; jAcol < 12; jAcol++) {
        temp = 0.0F;
        for (jp = 0; jp < 6; jp++) {
          temp += c_K_k[k + 12 * jp] * (float)g_a[jp + 6 * jAcol];
        }

        A_lin[k + 12 * jAcol] = (float)iv2[k + 12 * jAcol] - temp;
      }

      for (jAcol = 0; jAcol < 12; jAcol++) {
        P_apo[k + 12 * jAcol] = 0.0F;
        for (jp = 0; jp < 12; jp++) {
          P_apo[k + 12 * jAcol] += A_lin[k + 12 * jp] * P_apr[jp + 12 * jAcol];
        }
      }
    }
  } else {
    for (jy = 0; jy < 12; jy++) {
      x_apo[jy] = x_apr[jy];
    }

    memcpy(&P_apo[0], &P_apr[0], 144U * sizeof(float));
  }

  /* % euler anglels extraction */
  temp = norm(*(float (*)[3])&x_apo[6]);
  s = norm(*(float (*)[3])&x_apo[9]);
  for (jy = 0; jy < 3; jy++) {
    z_n_b[jy] = -x_apo[jy + 6] / temp;
    wak[jy] = x_apo[jy + 9] / s;
  }

  y_n_b[0] = z_n_b[1] * wak[2] - z_n_b[2] * wak[1];
  y_n_b[1] = z_n_b[2] * wak[0] - z_n_b[0] * wak[2];
  y_n_b[2] = z_n_b[0] * wak[1] - z_n_b[1] * wak[0];
  temp = norm(y_n_b);
  for (k = 0; k < 3; k++) {
    y_n_b[k] /= temp;
  }

  wak[0] = y_n_b[1] * z_n_b[2] - y_n_b[2] * z_n_b[1];
  wak[1] = y_n_b[2] * z_n_b[0] - y_n_b[0] * z_n_b[2];
  wak[2] = y_n_b[0] * z_n_b[1] - y_n_b[1] * z_n_b[0];
  temp = norm(wak);
  for (jy = 0; jy < 12; jy++) {
    xa_apo[jy] = x_apo[jy];
  }

  memcpy(&Pa_apo[0], &P_apo[0], 144U * sizeof(float));

  /*  rotation matrix from earth to body system */
  for (k = 0; k < 3; k++) {
    Rot_matrix[k] = wak[k] / temp;
    Rot_matrix[3 + k] = y_n_b[k];
    Rot_matrix[6 + k] = z_n_b[k];
  }

  eulerAngles[0] = -(90.0F + rt_atan2f_snf(Rot_matrix[8], Rot_matrix[6]) * 57.3F);
  eulerAngles[1] = -(float)asin(Rot_matrix[7]) * 57.3F;
  eulerAngles[2] = -rt_atan2f_snf(Rot_matrix[1], Rot_matrix[4]) * 57.3F;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void oldx_ekf_imu2_initialize(void)
{
  rt_InitInfAndNaN(8U);
  Q_not_empty = false;
  Ji_not_empty = false;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void oldx_ekf_imu2_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for oldx_ekf_imu2.c
 *
 * [EOF]
 */
