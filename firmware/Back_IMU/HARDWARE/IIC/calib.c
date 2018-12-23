/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#include "calib.h"
#include "LIS3MDL.h"
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

calibStruct_t calibData;

// solves U'*x = b for 3x3 matrix
void solveUT(float32_t *U, float32_t *b, float32_t *x) {
    float32_t t1 = b[1] - (U[1]*b[0]) / U[0];

    x[0] = b[0] / U[0];
    x[1] = t1 /  U[4];
    x[2] = -(U[5]*t1/U[4] - b[2] + U[2]*b[0]/U[0]) / U[8];
}

// solves U*-x = b for 3x3 matrix
void solveU(float32_t *U, float32_t *b, float32_t *x) {
    float32_t t1 = b[1] - (U[5]*b[2])/U[8];

    x[0] = +(U[1]*t1/U[4] - b[0] + U[2]*b[2]/U[8]) / U[0];
    x[1] = -(t1 / U[4]);
    x[2] = -(b[2] / U[8]);
}

int calibOctant(float32_t *vec) {
    float32_t x, y, z;

    x = vec[0] + calibData.bias[0];
    y = vec[1] + calibData.bias[1];
    z = vec[2] + calibData.bias[2];

    if (x > 0) {
        if (y > 0) {
            if (z > 0)
                return 0;
            else
                return 1;
        }
        else {
            if (z > 0)
                return 2;
            else
                return 3;
        }
    }
    else {
        if (y > 0) {
            if (z > 0)
                return 4;
            else
                return 5;
        }
        else {
            if (z > 0)
                return 6;
            else
                return 7;
        }
    }
}

// determine angle between two vectors
float32_t calibAngle(float *v1, float *v2) {
    float32_t v1b[3];
    float32_t v2b[3];
    float32_t t;
    int i;

    t = 0.0f;
    for (i = 0; i < 3; i++) {
        v1b[i] = v1[i] + calibData.bias[i];
        v2b[i] = v2[i] + calibData.bias[i];

        t += (v1b[i]*v2b[i]);
    }

    t /= __sqrtf(v1b[0]*v1b[0] + v1b[1]*v1b[1] + v1b[2]*v1b[2]);
    t /= __sqrtf(v2b[0]*v2b[0] + v2b[1]*v2b[1] + v2b[2]*v2b[2]);

    return (fabsf(t-1.0f) < 1e-6f) ? 0.0f : acosf(t);
}

void calibInsert(float32_t *v) {
    int idx;
    int i;

    idx = calibOctant(v) * CALIB_SAMPLES;

    for (i = 0; i < CALIB_SAMPLES; i++)
        if (calibData.calibSamples[(idx + i)*3 + 0] == CALIB_EMPTY_SLOT) {
            calibData.calibSamples[(idx + i)*3 + 0] = v[0];
            calibData.calibSamples[(idx + i)*3 + 1] = v[1];
            calibData.calibSamples[(idx + i)*3 + 2] = v[2];
            return;
        }
}

void calibRecalc() {
    int i, j, k;

    for (i = 0; i < 8; i++) {
        for (j = 0; j < CALIB_SAMPLES; j++) {
            k = i*CALIB_SAMPLES + j;

            if (calibData.calibSamples[k*3] != CALIB_EMPTY_SLOT) {
                if (calibOctant(&calibData.calibSamples[k*3]) != i) {
                    calibInsert(&calibData.calibSamples[k*3]);
                    calibData.calibSamples[k*3] = CALIB_EMPTY_SLOT;
                }
            }
        }
    }
}

int calibCount(void) {
    int sum;
    int i, j;

    sum = 0;
    for (i = 0; i < 8; i++)
        for (j = 0; j < CALIB_SAMPLES; j++)
            sum += (calibData.calibSamples[(i*CALIB_SAMPLES + j)*3] != CALIB_EMPTY_SLOT);

    calibData.percentComplete = (float32_t)sum / (8*CALIB_SAMPLES) * 100.0f;

    return sum;
}

void calibInit(void) {
    int i, j;

    calibData.calibSamples = (float32_t *)aqCalloc(CALIB_SAMPLES*8*3, sizeof(float32_t));

    for (i = 0; i < 3; i++) {
        calibData.min[i] = +999.9;
        calibData.max[i] = -999.9;
    }

    for (i = 0; i < 8; i++)
        for (j = 0; j < CALIB_SAMPLES; j++)
            calibData.calibSamples[(i*CALIB_SAMPLES + j)*3] = CALIB_EMPTY_SLOT;

    calibData.lastVec[0] = CALIB_EMPTY_SLOT;
    calibData.percentComplete = 0.0f;
}

void calibFree(void) {
    if (calibData.calibSamples) {
	aqFree(calibData.calibSamples, CALIB_SAMPLES*8*3, sizeof(float32_t));
        calibData.calibSamples = 0;
    }
}
void calibDeinit(void) {
    calibFree();
}

void calibCalculate(void) {
    arm_matrix_instance_f32 D, R;
    float32_t v[3];
    float32_t d, s;
    float32_t *t;
    float sign;
    int i;

    t = (float32_t *)aqCalloc(10*8*CALIB_SAMPLES, sizeof(float32_t));
    if (t == 0) {
//	AQ_NOTICE("CALIB: Out of memory\n");
	goto calibFree;
    }

    arm_mat_init_f32(&D, 10, 8*CALIB_SAMPLES, t);

    t = calibData.calibSamples;
    for (i = 0; i < 8*CALIB_SAMPLES; i++) {
	D.pData[0*8*CALIB_SAMPLES + i] = t[0]*t[0];
	D.pData[1*8*CALIB_SAMPLES + i] = t[1]*t[1];
	D.pData[2*8*CALIB_SAMPLES + i] = t[2]*t[2];
	D.pData[3*8*CALIB_SAMPLES + i] = t[0]*t[1];
	D.pData[4*8*CALIB_SAMPLES + i] = t[0]*t[2];
	D.pData[5*8*CALIB_SAMPLES + i] = t[1]*t[2];
	D.pData[6*8*CALIB_SAMPLES + i] = t[0];
	D.pData[7*8*CALIB_SAMPLES + i] = t[1];
	D.pData[8*8*CALIB_SAMPLES + i] = t[2];
	D.pData[9*8*CALIB_SAMPLES + i] = 1.0f;
	t += 3;
    }

    // free raw data
    calibFree();

    t = (float32_t *)aqCalloc(20*10, sizeof(float32_t));
    if (t == 0) {
	//AQ_NOTICE("CALIB: Out of memory\n");
	goto calibFree;
    }

    arm_mat_init_f32(&R, 20, 10, t);
    arm_fill_f32(0, t, 20*10);

    if (qrDecompositionT_f32(&D, 0, &R) == 0) {
	//AQ_NOTICE("CALIB: rank deficient - abort\n");
	goto calibFree;
    }

    svd(R.pData, calibData.U, 10);

    sign = (R.pData[10*10 + 9] < 0.0f) ? -1.0f : +1.0f;

    calibData.U[0] = sign * R.pData[(10+0)*10 + 9];
    calibData.U[1] = sign * R.pData[(10+3)*10 + 9] * 0.5f;
    calibData.U[2] = sign * R.pData[(10+4)*10 + 9] * 0.5f;
    calibData.U[3] = sign * R.pData[(10+3)*10 + 9] * 0.5f;
    calibData.U[4] = sign * R.pData[(10+1)*10 + 9];
    calibData.U[5] = sign * R.pData[(10+5)*10 + 9] * 0.5f;
    calibData.U[6] = sign * R.pData[(10+4)*10 + 9] * 0.5f;
    calibData.U[7] = sign * R.pData[(10+5)*10 + 9] * 0.5f;
    calibData.U[8] = sign * R.pData[(10+2)*10 + 9];

    calibData.bias[0] = sign * R.pData[(10+6)*10 + 9] * 0.5f;
    calibData.bias[1] = sign * R.pData[(10+7)*10 + 9] * 0.5f;
    calibData.bias[2] = sign * R.pData[(10+8)*10 + 9] * 0.5f;

    d = sign * R.pData[(10+9)*10 + 9];

    if (cholF(calibData.U) == 0) {
//	AQ_NOTICE("CALIB: poor data - abort\n");
	goto calibFree;
    }

    solveUT(calibData.U, calibData.bias, v);

    s = 1.0f / __sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] - d) * CALIB_SCALE;

    solveU(calibData.U, v, calibData.bias);

    for (i = 0; i < 9; i++)
	calibData.U[i] *= s;

    calibData.bias[0] = -calibData.bias[0];
    calibData.bias[1] = -calibData.bias[1];
    calibData.bias[2] = -calibData.bias[2];

    calibData.U[0] = 1.0f / calibData.U[0];
    calibData.U[4] = 1.0f / calibData.U[4];
    calibData.U[8] = 1.0f / calibData.U[8];
float p[10];
    // store config params
//    p[IMU_MAG_BIAS_X] = calibData.bias[0];
//    p[IMU_MAG_BIAS_Y] = calibData.bias[1];
//    p[IMU_MAG_BIAS_Z] = calibData.bias[2];
//    p[IMU_MAG_SCAL_X] = calibData.U[0];
//    p[IMU_MAG_SCAL_Y] = calibData.U[4];
//    p[IMU_MAG_SCAL_Z] = calibData.U[8];
//    p[IMU_MAG_ALGN_XY] = calibData.U[1];
//    p[IMU_MAG_ALGN_XZ] = calibData.U[2];
//    p[IMU_MAG_ALGN_YX] = calibData.U[3];
//    p[IMU_MAG_ALGN_YZ] = calibData.U[5];
//    p[IMU_MAG_ALGN_ZX] = calibData.U[6];
//    p[IMU_MAG_ALGN_ZY] = calibData.U[7];

    calibFree:

    if (D.pData)
	aqFree(D.pData, 10*8*CALIB_SAMPLES, sizeof(float32_t));
    if (R.pData)
	aqFree(R.pData, 20*10, sizeof(float32_t));
}

void calibFinished(void) {
    calibCalculate();
    //AQ_NOTICE("Calibration complete\n");
   // supervisorDisarm();
}

void calibrate(void) {
    float32_t vec[3];
    int i;

 
		vec[0] = lis3mdl.Mag_Adc.x;
		vec[1] = lis3mdl.Mag_Adc.y;
		vec[2] = lis3mdl.Mag_Adc.z;


    for (i = 0; i < 3; i++) {
        if (vec[i] < calibData.min[i])
            calibData.min[i] = vec[i];
        if (vec[i] > calibData.max[i])
            calibData.max[i] = vec[i];

        calibData.bias[i] = -(calibData.min[i] + calibData.max[i]) * 0.5f;

        calibRecalc();
    }

    if (calibData.lastVec[0] == CALIB_EMPTY_SLOT) {
        calibData.lastVec[0] = vec[0];
        calibData.lastVec[1] = vec[1];
        calibData.lastVec[2] = vec[2];
    }
    else if (calibAngle(calibData.lastVec, vec) * 57.3 > CALIB_MIN_ANGLE) {
	if (fabsf(calibData.min[0] - calibData.max[0]) > 1.0f &&
	    fabsf(calibData.min[1] - calibData.max[1]) > 1.0f &&
	    fabsf(calibData.min[2] - calibData.max[2]) > 1.0f) {
		calibInsert(vec);

		if (calibCount() == 8 * CALIB_SAMPLES)
		    calibFinished();

		calibData.lastVec[0] = vec[0];
		calibData.lastVec[1] = vec[1];
		calibData.lastVec[2] = vec[2];
	}
    }
}
