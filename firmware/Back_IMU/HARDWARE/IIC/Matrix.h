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

#ifndef _MATRIX_H_
#define _MATRIX_H_

#include "arm_math.h"

void arm_mat_zero_f32(arm_matrix_instance_f32* s);
arm_status mat_identity(float32_t *pData, uint16_t numRows, uint16_t numCols, float32_t value);
arm_status arm_mat_identity_f32(arm_matrix_instance_f32* s, float32_t value);
arm_status arm_mat_fill_f32(arm_matrix_instance_f32* s, float32_t *pData, uint32_t blockSize);
arm_status arm_mat_chol_f32(arm_matrix_instance_f32* s);
arm_status arm_mat_remainlower_f32(arm_matrix_instance_f32* s);

void arm_mat_getsubmatrix_f32(arm_matrix_instance_f32* s, arm_matrix_instance_f32 *a, int row, int col);
void arm_mat_setsubmatrix_f32(arm_matrix_instance_f32* a, arm_matrix_instance_f32 *s, int row, int col);

void arm_mat_getcolumn_f32(arm_matrix_instance_f32* s, float32_t *x, uint32_t col);
void arm_mat_setcolumn_f32(arm_matrix_instance_f32* s, float32_t *x, uint32_t col);
void arm_mat_cumsum_f32(arm_matrix_instance_f32* s, float32_t *tmp, float32_t *x);
int arm_mat_qr_decompositionT_f32(arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *R);

#endif
