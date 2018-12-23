/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MINIAHRS_H_
#define MINIAHRS_H_
//////////////////////////////////////////////////////////////////////////
//
#define EKF_STATE_DIM 7 //q0 q1 q2 q3 wxb wyb wzb
#define EKF_MEASUREMENT_DIM 6 //ax ay az and mx my mz

#define EKF_HALFPI 1.5707963267948966192313216916398f
#define EKF_PI 3.1415926535897932384626433832795f
#define EKF_TWOPI 6.283185307179586476925286766559f
#define EKF_TODEG(x) ((x) * 57.2957796f)

void EKF_AHRSInit(float *accel, float *mag);
void EKF_AHRSUpdate(float *gyro, float *accel, float *mag, float dt);
void EKF_AHRSGetAngle(float* rpy);

void EKF_AHRSGetQ(float* Q);

#endif
