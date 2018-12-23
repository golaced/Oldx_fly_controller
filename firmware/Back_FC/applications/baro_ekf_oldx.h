#ifndef __BARO_EKF_OLDX__
#define __BARO_EKF_OLDX__

#include <stddef.h>
#include <stdlib.h>
extern void BARO_EKF_OLDX(float x_apo[2], float P_apo[4],float xa_apo[2], float Pa_apo[4],const float z[2], float r_baro, float r_acc, float dt);
#endif
