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

#include "include.h"
#include "arm_math.h"
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

void matrixInit(arm_matrix_instance_f32 *m, int rows, int cols) {
    float32_t *d;

    d = (float32_t *)aqDataCalloc(rows*cols, sizeof(float32_t));

    arm_mat_init_f32(m, rows, cols, d);
    arm_fill_f32(0, d, rows*cols);
}

void matrixFree(arm_matrix_instance_f32 *m) {
    if (m && m->pData)
	free(m->pData);
}

// Calculates the QR decomposition of the given matrix A Transposed (decomp's A', not A)
//      notes:  A matrix is modified
//      Adapted from Java code originaly written by Joni Salonen
//
// returns 1 for success, 0 for failure
int qrDecompositionT_f32(arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R) {
    int minor;
    int row, col;
    int m = A->numCols;
    int n = A->numRows;
    int min;

    // clear R
    arm_fill_f32(0, R->pData, R->numRows*R->numCols);

    min = MIN(m, n);

    /*
    * The QR decomposition of a matrix A is calculated using Householder
    * reflectors by repeating the following operations to each minor
    * A(minor,minor) of A:
    */
    for (minor = 0; minor < min; minor++) {
	    float xNormSqr = 0.0f;
	    float a;

	    /*
	    * Let x be the first column of the minor, and a^2 = |x|^2.
	    * x will be in the positions A[minor][minor] through A[m][minor].
	    * The first column of the transformed minor will be (a,0,0,..)'
	    * The sign of a is chosen to be opposite to the sign of the first
	    * component of x. Let's find a:
	    */
	    for (row = minor; row < m; row++)
		    xNormSqr += A->pData[minor*m + row]*A->pData[minor*m + row];

	    a = __sqrtf(xNormSqr);
	    if (A->pData[minor*m + minor] > 0.0f)
		    a = -a;

	    if (a != 0.0f) {
		    R->pData[minor*R->numCols + minor] = a;

		    /*
		    * Calculate the normalized reflection vector v and transform
		    * the first column. We know the norm of v beforehand: v = x-ae
		    * so |v|^2 = <x-ae,x-ae> = <x,x>-2a<x,e>+a^2<e,e> =
		    * a^2+a^2-2a<x,e> = 2a*(a - <x,e>).
		    * Here <x, e> is now A[minor][minor].
		    * v = x-ae is stored in the column at A:
		    */
		    A->pData[minor*m + minor] -= a; // now |v|^2 = -2a*(A[minor][minor])

		    /*
		    * Transform the rest of the columns of the minor:
		    * They will be transformed by the matrix H = I-2vv'/|v|^2.
		    * If x is a column vector of the minor, then
		    * Hx = (I-2vv'/|v|^2)x = x-2vv'x/|v|^2 = x - 2<x,v>/|v|^2 v.
		    * Therefore the transformation is easily calculated by
		    * subtracting the column vector (2<x,v>/|v|^2)v from x.
		    *
		    * Let 2<x,v>/|v|^2 = alpha. From above we have
		    * |v|^2 = -2a*(A[minor][minor]), so
		    * alpha = -<x,v>/(a*A[minor][minor])
		    */
		    for (col = minor+1; col < n; col++) {
			    float alpha = 0.0f;

			    for (row = minor; row < m; row++)
				    alpha -= A->pData[col*m + row]*A->pData[minor*m + row];

			    alpha /= a*A->pData[minor*m + minor];

			    // Subtract the column vector alpha*v from x.
			    for (row = minor; row < m; row++)
				    A->pData[col*m + row] -= alpha*A->pData[minor*m + row];
		    }
	    }
	    // rank deficient
	    else
		return 0;
    }

    // Form the matrix R of the QR-decomposition.
    //      R is supposed to be m x n, but only calculate n x n
    // copy the upper triangle of A
    for (row = min-1; row >= 0; row--)
	    for (col = row+1; col < n; col++)
		    R->pData[row*R->numCols + col] = A->pData[col*m + row];

    // Form the matrix Q of the QR-decomposition.
    //      Q is supposed to be m x m

    // only compute Q if requested
    if (Q) {
	    arm_fill_f32(0, Q->pData, Q->numRows*Q->numCols);

	    /*
	    * Q = Q1 Q2 ... Q_m, so Q is formed by first constructing Q_m and then
	    * applying the Householder transformations Q_(m-1),Q_(m-2),...,Q1 in
	    * succession to the result
	    */
	    for (minor = m-1; minor >= min; minor--)
		    Q->pData[minor*m + minor] = 1.0f;

	    for (minor = min-1; minor >= 0; minor--) {
		    Q->pData[minor * m + minor] = 1.0f;

		    if (A->pData[minor*m + minor] != 0.0f) {
			    for (col = minor; col < m; col++) {
				    float alpha = 0.0f;

				    for (row = minor; row < m; row++)
					    alpha -= Q->pData[row*m + col]*A->pData[minor*m + row];

				    alpha /= R->pData[minor*R->numCols + minor]*A->pData[minor*m + minor];

				    for (row = minor; row < m; row++)
					    Q->pData[row*m + col] -= alpha*A->pData[minor*m + row];
			    }
		    }
	    }
    }

    return 1;
}

// Solves m sets of n equations B * X = A using QR decomposition and backsubstitution
void matrixDiv_f32(arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R, arm_matrix_instance_f32 *AQ) {
        int i, j, k;
        int m, n;

        // this is messy (going into a class's private data structure),
        // but it is better than malloc/free
        Q->numRows = B->numRows;
        Q->numCols = B->numRows;
        R->numRows = B->numRows;
        R->numCols = B->numCols;
        AQ->numRows = A->numRows;
        AQ->numCols = B->numRows;

        m = A->numRows;
        n = B->numCols;

        qrDecompositionT_f32(B, Q, R);
	arm_mat_mult_f32(A, Q, AQ);

        // solve for X by backsubstitution
        for (i = 0; i < m; i++) {
                for (j = n-1; j >= 0; j--) {
                        for (k = j+1; k < n; k++)
                                AQ->pData[i*n + j] -= R->pData[j*n + k] * X->pData[i*n + k];
                        X->pData[i*n + j] = AQ->pData[i*n + j] / R->pData[j*n + j];
                }
        }
}

void vectorNormalize(float32_t *v, int n) {
    float32_t t;
    int i;

    t = 0.0f;
    for (i = 0; i < n; i++)
	t += v[i] * v[i];

    t = __sqrtf(t);

    if (t > 1e-6f) {
	t = 1.0f / t;
	for (i = 0; i < n; i++)
	    v[i] *= t;
    }
    else {
	for (i = 0; i < n; i++)
	    v[i] *= 0.0f;
    }
}

// performs Cholesky factorization of 3x3 matrix
int cholF(float32_t *U) {
    float32_t a11 = U[0];
    float32_t a12 = U[1];
    float32_t a13 = U[2];
    float32_t a22 = U[4];
    float32_t a23 = U[5];
    float32_t a33 = U[8];

    float32_t t0 = 1.0f / __sqrtf(a11);
    float32_t t1 = a12 * t0;
    float32_t t2 = a13 * t0;
    float32_t t3 = t2 * t1;
    float32_t t4 = __sqrtf(a22 - t1*t1);
    float32_t t5 = (a23 - t3) / t4;

    U[0] = a11 * t0;
    U[1] = t1;
    U[2] = t2;

    U[3] = 0.0f;
    U[4] = t4;
    U[5] = t5;

    U[6] = 0.0f;
    U[7] = 0.0f;
    U[8] = __sqrtf(a33 - t3 - t5 * t5);

    // is positive definite?
    return (isnan(t4) || isnan(U[8])) ? 0 : 1;
}

/* svd.c: Perform a singular value decomposition A = USV' of square matrix.
 *
 * This routine has been adapted with permission from a Pascal implementation
 * (c) 1988 J. C. Nash, "Compact numerical methods for computers", Hilger 1990.
 * The A matrix must be pre-allocated with 2n rows and n columns. On calling
 * the matrix to be decomposed is contained in the first n rows of A. On return
 * the n first rows of A contain the product US and the lower n rows contain V
 * (not V'). The S2 vector returns the square of the singular values.
 *
 * (c) Copyright 1996 by Carl Edward Rasmussen. */

void svd(float32_t *A, float32_t *S2, int n)
{
  int  i, j, k, EstColRank = n, RotCount = n, SweepCount = 0,
       slimit = (n<120) ? 30 : n/4;
  float32_t eps = 1e-7, e2 = 10.0f*n*eps*eps, tol = 0.1f*eps, vt, p, x0,
       y0, q, r, c0, s0, d1, d2;

  for (i=0; i<n; i++) { for (j=0; j<n; j++) A[(n+i)*n + j] = 0.0; A[(n+i)*n + i] = 1.0; }
  while (RotCount != 0 && SweepCount++ <= slimit) {
    RotCount = EstColRank*(EstColRank-1)/2;
    for (j=0; j<EstColRank-1; j++)
      for (k=j+1; k<EstColRank; k++) {
        p = q = r = 0.0;
        for (i=0; i<n; i++) {
          x0 = A[i*n + j]; y0 = A[i*n + k];
          p += x0*y0; q += x0*x0; r += y0*y0;
        }
        S2[j] = q; S2[k] = r;
        if (q >= r) {
          if (q<=e2*S2[0] || fabsf(p)<=tol*q)
            RotCount--;
          else {
            p /= q; r = 1.0f-r/q; vt = __sqrtf(4.0f*p*p+r*r);
            c0 = __sqrtf(0.5f*(1.0f+r/vt)); s0 = p/(vt*c0);
            for (i=0; i<2*n; i++) {
              d1 = A[i*n + j]; d2 = A[i*n + k];
              A[i*n + j] = d1*c0+d2*s0; A[i*n + k] = -d1*s0+d2*c0;
            }
          }
        } else {
          p /= r; q = q/r-1.0f; vt = __sqrtf(4.0f*p*p+q*q);
          s0 = __sqrtf(0.5f*(1.0f-q/vt));
          if (p<0.0f) s0 = -s0;
          c0 = p/(vt*s0);
          for (i=0; i<2*n; i++) {
            d1 = A[i*n + j]; d2 = A[i*n + k];
            A[i*n + j] = d1*c0+d2*s0; A[i*n + k] = -d1*s0+d2*c0;
          }
        }
      }
    while (EstColRank>2 && S2[EstColRank-1]<=S2[0]*tol+tol*tol) EstColRank--;
  }
//  if (SweepCount > slimit)
//    printf("Warning: Reached maximum number of sweeps (%d) in SVD routine...\n"
//           ,slimit);
}
