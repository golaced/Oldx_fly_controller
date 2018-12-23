#ifndef __HEIGHTEKF_H__
#define __HEIGHTEKF_H__

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "include.h"

/* Function Declarations */
extern void HeightEKF(float x_apo[2], float P_apo[4], const unsigned char zFlag
                      [2], float dt, const float z[2], float r_baro, float r_acc,
                      float xa_apo[2], float Pa_apo[4]);
extern float X_apo_height[2];
#endif
