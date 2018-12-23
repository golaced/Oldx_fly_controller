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

#ifndef _calib_h
#define _calib_h

#include "include.h"
#include "arm_math.h"

#define CALIB_SAMPLES       20      // per octant
#define CALIB_MIN_ANGLE     25      // degrees
#define CALIB_EMPTY_SLOT    100.0f
#define CALIB_SCALE	    2.0f

typedef struct {
    float32_t *calibSamples;
    float32_t lastVec[3];
    float32_t min[3];
    float32_t max[3];
    float32_t bias[3];
    float32_t U[10];
    float32_t percentComplete;
} calibStruct_t;

extern calibStruct_t calibData;

extern void calibInit(void);
extern void calibDeinit(void);
extern void calibrate(void);

#endif
