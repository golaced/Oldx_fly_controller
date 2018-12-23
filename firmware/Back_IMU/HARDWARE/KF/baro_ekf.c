#include "baro_ekf.h"

/* Function Definitions */

/*
 * % init
 *      x_apo = X_apo_k;
 *      P_apo = P_apo_k;
 * Arguments    : float x_apo[2]
 *                float P_apo[4]
 *                const unsigned char zFlag[2]
 *                float dt
 *                const float z[2]
 *                float r_baro
 *                float r_acc
 *                float xa_apo[2]
 *                float Pa_apo[4]
 * Return Type  : void
 */
void HeightEKF(float x_apo[2], float P_apo[4], const unsigned char zFlag[2],
               float dt, const float z[2], float r_baro, float r_acc, float
               xa_apo[2], float Pa_apo[4])
{
  float A[4];
  int i0;
  float B[2];
  float H[2];
  float x_apr[2];
  float f0;
  int i1;
  double Q[4];
  float b_A[4];
  float c_A[4];
  float b_B[4];
  int j;
  float P_apr[4];
  float b_H[2];
  float y_k;
  float S_k;
  static const signed char iv0[4] = { 1, 0, 0, 1 };

  /* % copy the states */
  /* % prediction */
  A[0] = 1.0F;
  A[2] = dt;
  for (i0 = 0; i0 < 2; i0++) {
    A[1 + (i0 << 1)] = (float)i0;
  }

  B[0] = 0.5F * (dt * dt);
  B[1] = dt;
  H[0] = 1.0F;
  H[1] = dt;

  /*  x prediction ¨º¡À??¡ä?¦ÌY */
  for (i0 = 0; i0 < 2; i0++) {
    f0 = 0.0F;
    for (i1 = 0; i1 < 2; i1++) {
      f0 += A[i0 + (i1 << 1)] * x_apo[i1];
    }

    x_apr[i0] = f0 + B[i0] * z[1];
  }

  for (i0 = 0; i0 < 4; i0++) {
    Q[i0] = 0.0;
  }

  for (j = 0; j < 2; j++) {
    Q[j + (j << 1)] = 0.001;
    for (i0 = 0; i0 < 2; i0++) {
      b_A[j + (i0 << 1)] = 0.0F;
      for (i1 = 0; i1 < 2; i1++) {
        b_A[j + (i0 << 1)] += A[j + (i1 << 1)] * P_apo[i1 + (i0 << 1)];
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      c_A[j + (i0 << 1)] = 0.0F;
      for (i1 = 0; i1 < 2; i1++) {
        c_A[j + (i0 << 1)] += b_A[j + (i1 << 1)] * A[i0 + (i1 << 1)];
      }

      b_B[j + (i0 << 1)] = B[j] * r_acc * B[i0];
    }
  }

  for (i0 = 0; i0 < 2; i0++) {
    for (i1 = 0; i1 < 2; i1++) {
      P_apr[i1 + (i0 << 1)] = (c_A[i1 + (i0 << 1)] + b_B[i1 + (i0 << 1)]) +
        (float)Q[i1 + (i0 << 1)];
    }
  }

  /* % update */
  /*  baro+acc */
  if (zFlag[0] == 1) {
    f0 = 0.0F;
    for (i0 = 0; i0 < 2; i0++) {
      f0 += H[i0] * x_apr[i0];
      b_H[i0] = 0.0F;
      for (i1 = 0; i1 < 2; i1++) {
        b_H[i0] += H[i1] * P_apr[i1 + (i0 << 1)];
      }
    }

    y_k = z[0] - f0;
    f0 = 0.0F;
    for (i0 = 0; i0 < 2; i0++) {
      f0 += b_H[i0] * H[i0];
    }

    S_k = f0 + r_baro;
    for (i0 = 0; i0 < 2; i0++) {
      f0 = 0.0F;
      for (i1 = 0; i1 < 2; i1++) {
        f0 += P_apr[i0 + (i1 << 1)] * H[i1];
      }

      B[i0] = f0 / S_k;
    }

    for (j = 0; j < 2; j++) {
      x_apo[j] = x_apr[j] + B[j] * y_k;
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (i1 = 0; i1 < 2; i1++) {
        b_A[i0 + (i1 << 1)] = (float)iv0[i0 + (i1 << 1)] - B[i0] * H[i1];
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (i1 = 0; i1 < 2; i1++) {
        P_apo[i0 + (i1 << 1)] = 0.0F;
        for (j = 0; j < 2; j++) {
          P_apo[i0 + (i1 << 1)] += b_A[i0 + (j << 1)] * P_apr[j + (i1 << 1)];
        }
      }
    }
  } else {
    if (zFlag[0] == 0) {
      for (j = 0; j < 2; j++) {
        x_apo[j] = x_apr[j];
      }

      for (i0 = 0; i0 < 4; i0++) {
        P_apo[i0] = P_apr[i0];
      }
    }
  }

  for (j = 0; j < 2; j++) {
    xa_apo[j] = x_apo[j];
  }

  for (i0 = 0; i0 < 4; i0++) {
    Pa_apo[i0] = P_apo[i0];
  }
}
