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

#ifndef _MAXTRIX_H_
#define _MAXTRIX_H_

#include "FastMath.h"

//////////////////////////////////////////////////////////////////////////
//

void Matrix_Zero(float *A, unsigned short numRows, unsigned short numCols);
void Matrix_Copy(float *pSrc, unsigned short numRows, unsigned short numCols, float *pDst);
int Maxtrix_Add(float *pSrcA, unsigned short numRows, unsigned short numCols, float *pSrcB, float *pDst);
int Maxtrix_Sub(float *pSrcA, unsigned short numRows, unsigned short numCols, float *pSrcB, float *pDst);
int Matrix_Multiply(float* pSrcA, unsigned short numRowsA, unsigned short numColsA, float* pSrcB, unsigned short numColsB, float* pDst);
void Matrix_Multiply_With_Transpose(float *A, unsigned short nrows, unsigned short ncols, float *B, unsigned short mrows, float *C);
void Maxtrix_Transpose(float *pSrc, unsigned short nRows, unsigned short nCols, float *pDst);
int Matrix_Inverse(float* pDst, unsigned short n, float * pSrc);

#endif
