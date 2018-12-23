#include "alt_kf.h"
#include "include.h"
#include "imu.h"
#include "ms5611_2.h"
#include "string.h"
#include "height_ctrl.h"
#include "arm_math_m.h"
#include "stdlib.h"
#include "filter.h"
#include "baro_ekf.h"
#include "gps.h"
#include "nav_ukf.h"
altUkfStruct_t altUkfData,altUkfData_sonar,altUkfData_bmp;
altUkfStruct_t flowUkfData_x,flowUkfData_y;
altUkfStruct_t gpsUkfData_e,gpsUkfData_n;

arm_status arm_mat_trans_f32(
  const arm_matrix_instance_f32 * pSrc,
  arm_matrix_instance_f32 * pDst)
{
  float32_t *pIn = pSrc->pData;                  /* input data matrix pointer */
  float32_t *pOut = pDst->pData;                 /* output data matrix pointer */
  float32_t *px;                                 /* Temporary output data matrix pointer */
  uint16_t nRows = pSrc->numRows;                /* number of rows */
  uint16_t nColumns = pSrc->numCols;             /* number of columns */

#ifndef ARM_MATH_CM0

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  uint16_t blkCnt, i = 0u, row = nRows;          /* loop counters */
  arm_status status;                             /* status of matrix transpose  */


#ifdef ARM_MATH_MATRIX_CHECK


  /* Check for matrix mismatch condition */
  if((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
  {
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
  }
  else
#endif /*    #ifdef ARM_MATH_MATRIX_CHECK    */

  {
    /* Matrix transpose by exchanging the rows with columns */
    /* row loop     */
    do
    {
      /* Loop Unrolling */
      blkCnt = nColumns >> 2;

      /* The pointer px is set to starting address of the column being processed */
      px = pOut + i;

      /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.   
       ** a second loop below computes the remaining 1 to 3 samples. */
      while(blkCnt > 0u)        /* column loop */
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        blkCnt--;
      }

      /* Perform matrix transpose for last 3 samples here. */
      blkCnt = nColumns % 0x4u;

      while(blkCnt > 0u)
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        blkCnt--;
      }

#else

  /* Run the below code for Cortex-M0 */

  uint16_t col, i = 0u, row = nRows;             /* loop counters */
  arm_status status;                             /* status of matrix transpose  */


#ifdef ARM_MATH_MATRIX_CHECK

  /* Check for matrix mismatch condition */
  if((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
  {
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
  }
  else
#endif /*      #ifdef ARM_MATH_MATRIX_CHECK    */

  {
    /* Matrix transpose by exchanging the rows with columns */
    /* row loop     */
    do
    {
      /* The pointer px is set to starting address of the column being processed */
      px = pOut + i;

      /* Initialize column loop counter */
      col = nColumns;

      while(col > 0u)
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        col--;
      }

#endif /* #ifndef ARM_MATH_CM0 */

      i++;

      /* Decrement the row loop counter */
      row--;

    } while(row > 0u);          /* row loop end  */

    /* Set status as ARM_MATH_SUCCESS */
    status = ARM_MATH_SUCCESS;
  }

  /* Return to application */
  return (status);
}  
 
 
/**  
 * @brief Floating-point matrix multiplication.  
 * @param[in]       *pSrcA points to the first input matrix structure  
 * @param[in]       *pSrcB points to the second input matrix structure  
 * @param[out]      *pDst points to output matrix structure  
 * @return     The function returns either  
 * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.  
 */
 
arm_status arm_mat_mult_f32(
  const arm_matrix_instance_f32 * pSrcA,
  const arm_matrix_instance_f32 * pSrcB,
  arm_matrix_instance_f32 * pDst)
{
 
  float32_t *pIn1 = pSrcA->pData;                /* input data matrix pointer A */
  float32_t *pIn2 = pSrcB->pData;                /* input data matrix pointer B */
  float32_t *pInA = pSrcA->pData;                /* input data matrix pointer A  */
  float32_t *pOut = pDst->pData;                 /* output data matrix pointer */
  float32_t *px;                                 /* Temporary output data matrix pointer */
  float32_t sum;                                 /* Accumulator */
  uint16_t numRowsA = pSrcA->numRows;            /* number of rows of input matrix A */
  uint16_t numColsB = pSrcB->numCols;            /* number of columns of input matrix B */
  uint16_t numColsA = pSrcA->numCols;            /* number of columns of input matrix A */
 
#ifndef ARM_MATH_CM0
 
  /* Run the below code for Cortex-M4 and Cortex-M3 */
 
  uint16_t col, i = 0u, j, row = numRowsA, colCnt;      /* loop counters */
  arm_status status;                             /* status of matrix multiplication */
 
#ifdef ARM_MATH_MATRIX_CHECK
 
 
  /* Check for matrix mismatch condition */
  if((pSrcA->numCols != pSrcB->numRows) ||
     (pSrcA->numRows != pDst->numRows) || (pSrcB->numCols != pDst->numCols))
  {
 
 
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
   
}
  else
#endif /*      #ifdef ARM_MATH_MATRIX_CHECK    */
 
  {
 
    /* The following loop performs the dot-product of each row in pSrcA with each column in pSrcB */
    /* row loop */
    do
    {
 
      /* Output pointer is set to starting address of the row being processed */
      px = pOut + i;
 
      /* For every row wise process, the column loop counter is to be initiated */
      col = numColsB;
 
      /* For every row wise process, the pIn2 pointer is set  
       ** to the starting address of the pSrcB data */
      pIn2 = pSrcB->pData;
 
      j = 0u;
 
      /* column loop */
      do
      {
 
        /* Set the variable sum, that acts as accumulator, to zero */
        sum = 0.0f;
 
        /* Initiate the pointer pIn1 to point to the starting address of the column being processed */
        pIn1 = pInA;
 
        /* Apply loop unrolling and compute 4 MACs simultaneously. */
        colCnt = numColsA >> 2;
 
        /* matrix multiplication        */
        while(colCnt > 0u)
        {
 
          /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;
 
          /* Decrement the loop count */
          colCnt--;
         
}
 
        /* If the columns of pSrcA is not a multiple of 4, compute any remaining MACs here.  
         ** No loop unrolling is used. */
        colCnt = numColsA % 0x4u;
 
        while(colCnt > 0u)
        {
 
          /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;
 
          /* Decrement the loop counter */
          colCnt--;
         
}
 
        /* Store the result in the destination buffer */
        *px++ = sum;
 
        /* Update the pointer pIn2 to point to the  starting address of the next column */
        j++;
        pIn2 = pSrcB->pData + j;
 
        /* Decrement the column loop counter */
        col--;
 
       
} while(col > 0u);
 
#else
 
  /* Run the below code for Cortex-M0 */
 
  float32_t *pInB = pSrcB->pData;                /* input data matrix pointer B */
  uint16_t col, i = 0u, row = numRowsA, colCnt;  /* loop counters */
  arm_status status;                             /* status of matrix multiplication */
 
#ifdef ARM_MATH_MATRIX_CHECK
 
  /* Check for matrix mismatch condition */
  if((pSrcA->numCols != pSrcB->numRows) ||
     (pSrcA->numRows != pDst->numRows) || (pSrcB->numCols != pDst->numCols))
  {
 
 
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
   
}
  else
#endif /*      #ifdef ARM_MATH_MATRIX_CHECK    */
 
  {
 
    /* The following loop performs the dot-product of each row in pInA with each column in pInB */
    /* row loop */
    do
    {
 
      /* Output pointer is set to starting address of the row being processed */
      px = pOut + i;
 
      /* For every row wise process, the column loop counter is to be initiated */
      col = numColsB;
 
      /* For every row wise process, the pIn2 pointer is set   
       ** to the starting address of the pSrcB data */
      pIn2 = pSrcB->pData;
 
      /* column loop */
      do
      {
 
        /* Set the variable sum, that acts as accumulator, to zero */
        sum = 0.0f;
 
        /* Initialize the pointer pIn1 to point to the starting address of the row being processed */
        pIn1 = pInA;
 
        /* Matrix A columns number of MAC operations are to be performed */
        colCnt = numColsA;
 
        while(colCnt > 0u)
        {
 
          /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;
 
          /* Decrement the loop counter */
          colCnt--;
         
}
 
        /* Store the result in the destination buffer */
        *px++ = sum;
 
        /* Decrement the column loop counter */
        col--;
 
        /* Update the pointer pIn2 to point to the  starting address of the next column */
        pIn2 = pInB + (numColsB - col);
 
       
} while(col > 0u);
 
#endif /* #ifndef ARM_MATH_CM0 */
 
      /* Update the pointer pInA to point to the  starting address of the next row */
      i = i + numColsB;
      pInA = pInA + numColsA;
 
      /* Decrement the row loop counter */
      row--;
 
     
} while(row > 0u);
    /* Set status as ARM_MATH_SUCCESS */
    status = ARM_MATH_SUCCESS;
   
}
 
  /* Return to application */
  return (status);
 
}
 

/**  
   * @brief  Floating-point matrix initialization.  
   * @param[in,out] *S             points to an instance of the floating-point matrix structure.  
   * @param[in]     nRows          number of rows in the matrix.  
   * @param[in]     nColumns       number of columns in the matrix.  
   * @param[in]     *pData   points to the matrix data array.  
   * @return        none  
   */
 
void arm_mat_init_f32(
  arm_matrix_instance_f32 * S,
  uint16_t nRows,
  uint16_t nColumns,
  float32_t * pData)
{
 
  /* Assign Number of Rows */
  S->numRows = nRows;
 
  /* Assign Number of Columns */
  S->numCols = nColumns;
 
  /* Assign Data pointer */
  S->pData = pData;
 
}
 

/**  
 * @brief Fills a constant value into a floating-point vector.   
 * @param[in]       value input value to be filled 
 * @param[out]      *pDst points to output vector  
 * @param[in]       blockSize length of the output vector 
 * @return none.  
 *  
 */
 
 
void arm_fill_f32(
  float32_t value,
  float32_t * pDst,
  uint32_t blockSize)
{
 
  uint32_t blkCnt;                               /* loop counter */
 
#ifndef ARM_MATH_CM0
 
  /* Run the below code for Cortex-M4 and Cortex-M3 */
 
  /*loop Unrolling */
  blkCnt = blockSize >> 2u;
 
  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.  
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
 
    /* C = value */
    /* Fill the value in the destination buffer */
    *pDst++ = value;
    *pDst++ = value;
    *pDst++ = value;
    *pDst++ = value;
 
    /* Decrement the loop counter */
    blkCnt--;
   
}
 
  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.  
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;
 
#else
 
  /* Run the below code for Cortex-M0 */
 
  /* Loop over blockSize number of values */
  blkCnt = blockSize;
 
#endif /* #ifndef ARM_MATH_CM0 */
 
 
  while(blkCnt > 0u)
  {
 
    /* C = value */
    /* Fill the value in the destination buffer */
    *pDst++ = value;
 
    /* Decrement the loop counter */
    blkCnt--;
   
}
 
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
#define UTIL_CCM_HEAP_SIZE	    (0x2800)	// 40KB
uint32_t heapUsed, heapHighWater, dataSramUsed;
uint32_t *ccmHeap[UTIL_CCM_HEAP_SIZE] __attribute__((section(".ccm")));

// allocates memory from 64KB CCM
void *aqDataCalloc(uint16_t count, uint16_t size) {
    uint32_t words;

    // round up to word size
    words = (count*size + sizeof(int)-1) / sizeof(int);

    if ((dataSramUsed + words) > UTIL_CCM_HEAP_SIZE) {
       ;// AQ_NOTICE("Out of data SRAM!\n");
    }
    else {
        dataSramUsed += words;
    }

    return (void *)(ccmHeap + dataSramUsed - words);
}

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
//given noise matrix  生成采样点
static void srcdkfCalcSigmaPoints(srcdkf_t *f, arm_matrix_instance_f32 *Sn) {
	int S = f->S;			// number of states
	int N = Sn->numRows;		// number of noise variables
	int A = S+N;			// number of agumented states
	int L = 1+A*2;			// number of sigma points
	float32_t *x = f->x.pData;	// state
	float32_t *Sx = f->Sx.pData;	// state covariance
	float32_t *Xa = f->Xa.pData;	// augmented sigma points
	int i, j;

	// set the number of sigma points
	f->L = L;

	// resize output matrix
	f->Xa.numRows = A;
	f->Xa.numCols = L;

	//	-	   -
	// Sa =	| Sx	0  |
	//	| 0	Sn |
	//	-	   -
	// xa = [ x 	0  ]
	// Xa = [ xa  (xa + h*Sa)  (xa - h*Sa) ]
	//
	for (i = 0; i < A; i++) {
		int rOffset = i*L;
		float32_t base = (i < S) ? x[i] : 0.0f;

		Xa[rOffset + 0] = base;

		for (j = 1; j <= A; j++) {
			float32_t t = 0.0f;

			if (i < S && j < S+1)
				t = Sx[i*S + (j-1)]*f->h;

			if (i >= S && j >= S+1)
				t = Sn->pData[(i-S)*N + (j-S-1)]*f->h;

			Xa[rOffset + j]     = base + t;
			Xa[rOffset + j + A] = base - t;
		}
	}
}

void srcdkfTimeUpdate(srcdkf_t *f, float32_t *u, float32_t dt) {
	int S = f->S;			// number of states
	int V = f->V;			// number of noise variables
	int L;				// number of sigma points
	float32_t *x = f->x.pData;	// state estimate
	float32_t *Xa = f->Xa.pData;	// augmented sigma points
//	float32_t *xIn = f->xIn;	// callback buffer
//	float32_t *xOut = f->xOut;	// callback buffer
//	float32_t *xNoise = f->xNoise;	// callback buffer
	float32_t *qrTempS = f->qrTempS.pData;
	int i, j;
 //技术sima点
	srcdkfCalcSigmaPoints(f, &f->Sv);
	L = f->L;

	// Xa = f(Xx, Xv, u, dt)
//	for (i = 0; i < L; i++) {
//		for (j = 0; j < S; j++)
//			xIn[j] = Xa[j*L + i];
//
//		for (j = 0; j < V; j++)
//			xNoise[j] = Xa[(S+j)*L + i];
//
//		f->timeUpdate(xIn, xNoise, xOut, u, dt);
//
//		for (j = 0; j < S; j++)
//			Xa[j*L + i] = xOut[j];
//	}
  //对sima点 非线性变化
	f->timeUpdate(&Xa[0], &Xa[S*L], &Xa[0], u, dt, L);
  //求取状态权值
	// sum weighted resultant sigma points to create estimated state
	f->w0m = (f->hh - (float32_t)(S+V)) / f->hh;
	for (i = 0; i < S; i++) {
		int rOffset = i*L;

		x[i] = Xa[rOffset + 0] * f->w0m;

		for (j = 1; j < L; j++)
			x[i] += Xa[rOffset + j] * f->wim;
	}
  
	// update state covariance
	for (i = 0; i < S; i++) {
		int rOffset = i*(S+V)*2;

		for (j = 0; j < S+V; j++) {
			qrTempS[rOffset + j] = (Xa[i*L + j + 1] - Xa[i*L + S+V + j + 1]) * f->wic1;
			qrTempS[rOffset + S+V + j] = (Xa[i*L + j + 1] + Xa[i*L + S+V + j + 1] - 2.0f*Xa[i*L + 0]) * f->wic2;
		}
	}

	qrDecompositionT_f32(&f->qrTempS, NULL, &f->SxT);   // with transposition
	arm_mat_trans_f32(&f->SxT, &f->Sx);
}

void srcdkfMeasurementUpdate(srcdkf_t *f, float32_t *u, float32_t *ym, int M, int N, float32_t *noise, SRCDKFMeasurementUpdate_t *measurementUpdate) {
	int S = f->S;				// number of states
	float32_t *Xa = f->Xa.pData;			// sigma points
	float32_t *xIn = f->xIn;			// callback buffer
	float32_t *xNoise = f->xNoise;		// callback buffer
	float32_t *xOut = f->xOut;			// callback buffer
	float32_t *Y = f->Y.pData;			// measurements from sigma points
	float32_t *y = f->y.pData;			// measurement estimate
	float32_t *Sn = f->Sn.pData;			// observation noise covariance
	float32_t *qrTempM = f->qrTempM.pData;
	float32_t *C1 = f->C1.pData;
	float32_t *C1T = f->C1T.pData;
	float32_t *C2 = f->C2.pData;
	float32_t *D = f->D.pData;
	float32_t *inov = f->inov.pData;		// M x 1 matrix
	float32_t *xUpdate = f->xUpdate.pData;	// S x 1 matrix
	float32_t *x = f->x.pData;			// state estimate
	float32_t *Sx = f->Sx.pData;
	float32_t *Q = f->Q.pData;
	float32_t *qrFinal = f->qrFinal.pData;
	int L;					// number of sigma points
	int i, j;

	// make measurement noise matrix if provided
	if (noise) {
		f->Sn.numRows = N;
		f->Sn.numCols = N;
		arm_fill_f32(0.0f, f->Sn.pData, N*N);
		for (i = 0; i < N; i++)
			arm_sqrt_f32(fabsf(noise[i]), &Sn[i*N + i]);
	}

	// generate sigma points
	srcdkfCalcSigmaPoints(f, &f->Sn);
	L = f->L;

	// resize all N and M based storage as they can change each iteration
	f->y.numRows = M;
	f->Y.numRows = M;
	f->Y.numCols = L;
	f->qrTempM.numRows = M;
	f->qrTempM.numCols = (S+N)*2;
	f->Sy.numRows = M;
	f->Sy.numCols = M;
	f->SyT.numRows = M;
	f->SyT.numCols = M;
	f->SyC.numRows = M;
	f->SyC.numCols = M;
	f->Pxy.numCols = M;
	f->C1.numRows = M;
	f->C1T.numCols = M;
	f->C2.numRows = M;
	f->C2.numCols = N;
	f->D.numRows = M;
	f->D.numCols = S+N;
	f->K.numCols = M;
	f->inov.numRows = M;
	f->qrFinal.numCols = 2*S + 2*N;

	// Y = h(Xa, Xn)
	for (i = 0; i < L; i++) {
		for (j = 0; j < S; j++)
			xIn[j] = Xa[j*L + i];

		for (j = 0; j < N; j++)
			xNoise[j] = Xa[(S+j)*L + i];

		measurementUpdate(u, xIn, xNoise, xOut);

		for (j = 0; j < M; j++)
			Y[j*L + i] = xOut[j];
	}

	// sum weighted resultant sigma points to create estimated measurement
	f->w0m = (f->hh - (float32_t)(S+N)) / f->hh;
	for (i = 0; i < M; i++) {
		int rOffset = i*L;

		y[i] = Y[rOffset + 0] * f->w0m;

		for (j = 1; j < L; j++)
			y[i] += Y[rOffset + j] * f->wim;
	}

	// calculate measurement covariance components
	for (i = 0; i < M; i++) {
		int rOffset = i*(S+N)*2;

		for (j = 0; j < S+N; j++) {
			float32_t c, d;

			c = (Y[i*L + j + 1] - Y[i*L + S+N + j + 1]) * f->wic1;
			d = (Y[i*L + j + 1] + Y[i*L + S+N + j + 1] - 2.0f*Y[i*L]) * f->wic2;

			qrTempM[rOffset + j] = c;
			qrTempM[rOffset + S+N + j] = d;

			// save fragments for future operations
			if (j < S) {
				C1[i*S + j] = c;
				C1T[j*M + i] = c;
			}
			else {
				C2[i*N + (j-S)] = c;
			}
			D[i*(S+N) + j] = d;
		}
	}

	qrDecompositionT_f32(&f->qrTempM, NULL, &f->SyT);	// with transposition

	arm_mat_trans_f32(&f->SyT, &f->Sy);
	arm_mat_trans_f32(&f->SyT, &f->SyC);		// make copy as later Div is destructive

	// create Pxy
	arm_mat_mult_f32(&f->Sx, &f->C1T, &f->Pxy);
  //kalman 增益
	// K = (Pxy / SyT) / Sy
	matrixDiv_f32(&f->K, &f->Pxy, &f->SyT, &f->Q, &f->R, &f->AQ);
	matrixDiv_f32(&f->K, &f->K, &f->Sy, &f->Q, &f->R, &f->AQ);
  //修正状态估计
	// x = x + k(ym - y)
	for (i = 0; i < M; i++)
		inov[i] = ym[i] - y[i];
	arm_mat_mult_f32(&f->K, &f->inov, &f->xUpdate);

	for (i = 0; i < S; i++)
		x[i] += xUpdate[i];

	// build final QR matrix
	//	rows = s
	//	cols = s + n + s + n
	//	use Q as temporary result storage

	f->Q.numRows = S;
	f->Q.numCols = S;
	arm_mat_mult_f32(&f->K, &f->C1, &f->Q);
	for (i = 0; i < S; i++) {
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < S; j++)
			qrFinal[rOffset + j] = Sx[i*S + j] - Q[i*S + j];
	}

	f->Q.numRows = S;
	f->Q.numCols = N;
	arm_mat_mult_f32(&f->K, &f->C2, &f->Q);
	for (i = 0; i < S; i++) {
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < N; j++)
			qrFinal[rOffset + S+j] = Q[i*N + j];
	}

	f->Q.numRows = S;
	f->Q.numCols = S+N;
	arm_mat_mult_f32(&f->K, &f->D, &f->Q);
	for (i = 0; i < S; i++) {
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < S+N; j++)
			qrFinal[rOffset + S+N+j] = Q[i*(S+N) + j];
	}

	// Sx = qr([Sx-K*C1 K*C2 K*D]')
	// this method is not susceptable to numeric instability like the Cholesky is
	qrDecompositionT_f32(&f->qrFinal, NULL, &f->SxT);	// with transposition
	arm_mat_trans_f32(&f->SxT, &f->Sx);
}

// states, max observations, process noise, max observation noise
srcdkf_t *srcdkfInit(int s, int m, int v, int n, SRCDKFTimeUpdate_t *timeUpdate) {
	srcdkf_t *f;
	int maxN = MAX(v, n);

	f = (srcdkf_t *)aqDataCalloc(1, sizeof(srcdkf_t));

	f->S = s;
	f->V = v;

	matrixInit(&f->Sx, s, s);
	matrixInit(&f->SxT, s, s);
	matrixInit(&f->Sv, v, v);
	matrixInit(&f->Sn, n, n);
	matrixInit(&f->x, s, 1);
	matrixInit(&f->Xa, s+maxN, 1+(s+maxN)*2);

	matrixInit(&f->qrTempS, s, (s+v)*2);
	matrixInit(&f->y, m, 1);
	matrixInit(&f->Y, m, 1+(s+n)*2);
	matrixInit(&f->qrTempM, m, (s+n)*2);
	matrixInit(&f->Sy, m, m);
	matrixInit(&f->SyT, m, m);
	matrixInit(&f->SyC, m, m);
	matrixInit(&f->Pxy, s, m);
	matrixInit(&f->C1, m, s);
	matrixInit(&f->C1T, s, m);
	matrixInit(&f->C2, m, n);
	matrixInit(&f->D, m, s+n);
	matrixInit(&f->K, s, m);
	matrixInit(&f->inov, m, 1);
	matrixInit(&f->xUpdate, s, 1);
	matrixInit(&f->qrFinal, s, 2*s + 2*n);
	matrixInit(&f->Q, s, s+n);	// scratch
	matrixInit(&f->R, n, n);	// scratch
	matrixInit(&f->AQ, s, n);	// scratch

	f->xOut = (float32_t *)aqDataCalloc(s, sizeof(float32_t));
	f->xNoise = (float32_t *)aqDataCalloc(maxN, sizeof(float32_t));
	f->xIn = (float32_t *)aqDataCalloc(s, sizeof(float32_t));

	f->h = SRCDKF_H;
	f->hh = f->h*f->h;
//	f->w0m = (f->hh - (float32_t)s) / f->hh;	// calculated in process
	f->wim = 1.0f / (2.0f * f->hh);
	f->wic1 = __sqrtf(1.0f / (4.0f * f->hh));
	f->wic2 = __sqrtf((f->hh - 1.0f) / (4.0f * f->hh*f->hh));

        f->timeUpdate = timeUpdate;

	return f;
}

float *srcdkfGetState(srcdkf_t *f) {
    return f->x.pData;
}
void srcdkfSetVariance(srcdkf_t *f, float32_t *q, float32_t *v, float32_t *n, int nn) {
	float32_t *Sx = f->Sx.pData;
	float32_t *Sv = f->Sv.pData;
	float32_t *Sn = f->Sn.pData;
	int i;

	// state variance
	if (q)
		for (i = 0; i < f->S; i++)
			Sx[i*f->S + i] = __sqrtf(fabsf(q[i]));

	// process noise
	if (v)
		for (i = 0; i < f->V; i++)
			Sv[i*f->V + i] = __sqrtf(fabsf(v[i]));

	// observation noise
	if (n && nn) {
		// resize Sn
		f->Sn.numRows = nn;
		f->Sn.numCols = nn;

		for (i = 0; i < nn; i++)
			Sn[i*nn + i] = __sqrtf(fabsf(n[i]));
	}
}


void altUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) {
    float acc;
    int i;
    out = in;

    for (i = 0; i < n; i++) {
        acc = u[0] + in[ALT_STATE_BIAS*n + i];//修正加速度偏差
//更新状态方程
        out[ALT_STATE_BIAS*n + i] = in[ALT_STATE_BIAS*n + i] + (noise[ALT_NOISE_BIAS*n + i] * dt);
        out[ALT_STATE_VEL*n + i] = in[ALT_STATE_VEL*n + i] + (acc * dt) + (noise[ALT_NOISE_VEL*n + i] * dt);
        out[ALT_STATE_POS*n + i] = in[ALT_STATE_POS*n + i] - (in[ALT_STATE_VEL*n + i] * dt) - (acc * dt * dt * 0.5f);
    }
}

void flowUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) {
    float acc;
    int i;
    out = in;

    for (i = 0; i < n; i++) {
        acc = u[0] + in[ALT_STATE_BIAS*n + i];

        out[ALT_STATE_BIAS*n + i] = in[ALT_STATE_BIAS*n + i] + (noise[ALT_NOISE_BIAS*n + i] * dt);
        out[ALT_STATE_VEL*n + i] = in[ALT_STATE_VEL*n + i] + (acc * dt) + (noise[ALT_NOISE_VEL*n + i] * dt);
        out[ALT_STATE_POS*n + i] = in[ALT_STATE_POS*n + i] - (in[ALT_STATE_VEL*n + i] * dt) - (acc * dt * dt * 0.5f);
    }
}
//	measurementUpdate(u, xIn, xNoise, xOut);
void altUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[ALT_STATE_POS] + noise[0];     // return altitude
}
void flowUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[ALT_STATE_VEL] + noise[0];     // return altitude
}
void gpsUpdate(float *u, float *x, float *noise, float *y) {
   //y[0] = x[ALT_STATE_VEL] + noise[0];     // return altitude
	 y[0] = x[ALT_STATE_POS] + noise[0];     // return altitude
}

//flow
float FLOW_NOISE =0.008;//2.00000011e-005;//0.0003;//.00000011e-005;//5e-5f;//5e-4f;//0.00052f;
float flow_pos[2];
static void flow_ukf_Update(float dt) {
    float noise;        // measurement variance
    float y[2];            // measurment
    flowUkfData_x.x = srcdkfGetState(flowUkfData_x.kf);
		flowUkfData_y.x = srcdkfGetState(flowUkfData_y.kf);

static u16 init_cnt;
	if(init_cnt++>100){init_cnt=101;

	y[0] = flow_matlab_data[2];
	y[1] = flow_matlab_data[3];
	noise = FLOW_NOISE;
	srcdkfMeasurementUpdate(flowUkfData_x.kf, 0, &y[0], 1, 1, &noise, flowUpdate);
	srcdkfMeasurementUpdate(flowUkfData_y.kf, 0, &y[1], 1, 1, &noise, flowUpdate);
	}
} 
u16 Distance[50]  = {0};
u8 measure_num=8;
float sonar_filter_oldx(float in) 
{
   
    u8 IS_success =1;
    u16 Distance1 = 0;
    u16 MAX_error1 = 0;
    u8 MAX_error_targe = 0;
    u8 count = 0;
    u8 i =0;
    u8 j =0;
    u8 num =0;  //实际只测到的次数
    Distance[measure_num-1]=in*1000;
    for(i=0;i<measure_num-1;i++)
		{
		 Distance[i]=Distance[i+1]; 
		}

   
         //排序
        for(i = 0 ; i < measure_num-1 ; i++)
        {

            for(j = 0 ; j < measure_num-1-i; j++)       
            {
                if(Distance[j] > Distance[j+1] )
                {
                    Distance1 = Distance[j];
                    Distance[j] =  Distance[j+1];
                    Distance[j+1] = Distance1; 
                }
            }

        }

        //找出最大差距
        MAX_error1 = Distance[1] - Distance[0];
        for(i = 1 ; i < measure_num-1 ; i++)
        {

            if(MAX_error1 < Distance[i+1] - Distance[i] )//如：1 2 3 4 5    8 9 10    MAX_error_targe=4;
            {
                MAX_error1 =  Distance[i+1] - Distance[i];//最大差距
                MAX_error_targe = i;  //记下最大差距值的位置（这组数中的位置）
            }

        }
        float UltrasonicWave_Distance1=0;
        //取出最终值
        if(MAX_error_targe+1 > (measure_num+1)/2) //前部分有效  1 2 3 4 5    8 9 10  (如果位于中间，后半部优先)
        {
            for(i = 0 ; i <= MAX_error_targe ; i++)
            {
                UltrasonicWave_Distance1 += Distance[i];
            }

            UltrasonicWave_Distance1 /= (MAX_error_targe+1);//取平均

        }
        else  //后部分有效  1 2 3   7 8 9 10
        {
             for(i = MAX_error_targe + 1 ; i < measure_num ; i++)
            {
                UltrasonicWave_Distance1 += Distance[i];
            }

            UltrasonicWave_Distance1 /= (measure_num - MAX_error_targe -1);//取平均

        }


    return  (float)UltrasonicWave_Distance1/1000.; //转化为米为单位的浮点数
}



static float sonar_values[3] = { 0.0f };
static unsigned insert_index = 0;

static void sonar_bubble_sort(float sonar_values[], unsigned n);

void sonar_bubble_sort(float sonar_values[], unsigned n)
{
	float t;

	for (unsigned i = 0; i < (n - 1); i++) {
		for (unsigned j = 0; j < (n - i - 1); j++) {
			if (sonar_values[j] > sonar_values[j+1]) {
				/* swap two values */
				t = sonar_values[j];
				sonar_values[j] = sonar_values[j + 1];
				sonar_values[j + 1] = t;
			}
		}
	}
}

float insert_sonar_value_and_get_mode_value(float insert)
{
	const unsigned sonar_count = sizeof(sonar_values) / sizeof(sonar_values[0]);

	sonar_values[insert_index] = insert;
	insert_index++;
	if (insert_index == sonar_count) {
		insert_index = 0;
	}

	/* sort and return mode */

	/* copy ring buffer */
	float sonar_temp[sonar_count];
	memcpy(sonar_temp, sonar_values, sizeof(sonar_values));

	sonar_bubble_sort(sonar_temp, sonar_count);

	/* the center element represents the mode after sorting */
	return sonar_temp[sonar_count / 2];
}

void fusion_prepare(float dT,float av_arr[],u16 av_num,u16 *av_cnt,float deadzone,_height_st *data,_fusion_p_st *pre_data)
{
	pre_data->dis_deadzone = my_deathzoom1(data->relative_height,pre_data->dis_deadzone,deadzone);	
	Moving_Average1(av_arr,av_num ,(av_cnt),(10 *pre_data->dis_deadzone ),&(pre_data->displacement)); //厘米->毫米
	
//	Moving_Average(av_arr,av_num ,(av_cnt),(10 *data->relative_height),&(pre_data->dis_deadzone)); //厘米->毫米	
//	pre_data->displacement = my_deathzoom(pre_data->dis_deadzone,pre_data->displacement,10 *deadzone);
	
	pre_data->speed = safe_div(pre_data->displacement - pre_data->displacement_old,dT,0);
	pre_data->acceleration = safe_div(pre_data->speed - pre_data->speed_old,dT,0);
	
	
	pre_data->displacement_old = pre_data->displacement;
	pre_data->speed_old = pre_data->speed;
}
float k_acc=1;
void acc_fusion(float dT,_f_set_st *set,float est_acc,_fusion_p_st *pre_data,_fusion_st *fusion)
{
	fusion->fusion_acceleration.out += est_acc/k_acc - fusion->est_acc_old; //估计
	anotc_filter_1(set->b1,set->g1,dT,pre_data->acceleration,&(fusion->fusion_acceleration));  //pre_data->acceleration //观测、最优
	
	fusion->fusion_speed_m.out += 1.1f *my_deathzoom1(fusion->fusion_acceleration.out,0,20) *dT;
	anotc_filter_1(set->b2,set->g2,dT,pre_data->speed,&(fusion->fusion_speed_m));
	anotc_filter_1(set->b2,set->g2,dT,(-pre_data->speed + fusion->fusion_speed_m.out),&(fusion->fusion_speed_me));
	fusion->fusion_speed_me.out = LIMIT(fusion->fusion_speed_me.out,-200,200);
	fusion->fusion_speed_m.a = LIMIT(fusion->fusion_speed_m.a,-1000,1000);
	
	fusion->fusion_displacement.out += 1.05f *(fusion->fusion_speed_m.out - fusion->fusion_speed_me.out) *dT;
	anotc_filter_1(set->b3,set->g3,dT,pre_data->displacement,&(fusion->fusion_displacement));
	
	fusion->est_acc_old = est_acc;
}
#define SONAR_AV_NUM 5
_height_st ultra;
float sonar_av_arr[SONAR_AV_NUM];
u16 sonar_av_cnt;
_fusion_p_st sonar;
_fusion_st sonar_fusion;
_f_set_st sonar_f_set = {
													0.2f,
													0.5f,
													0.8f,
													
													0.2f,
													0.5f,
													0.8f	
//													0.2f,
//													0.3f,
//													0.5f,
//													
//													0.1f,
//													0.3f,
//													0.5f

												};


//alt
float ALT_PRES_NOISE = 0.001f;
float ALT_PRES_NOISE_SONAR = 0.0002;//015f;
#define USE_UKF_SONAR 0
float ALT_POS_SONAR2,ALT_POS_SONAR3;
#define SONAR_HIHG_NUM 10
float sonar_h_arr[SONAR_HIHG_NUM + 1];
u16 sonar_h_cnt[2];	 float sonar_temp;
float K_SONAR=3.6;
u16 test_1=1200;
u8 en_1=1;
float NOISE[3]={0.02,0.02,0.02};//{0.075,0.075,0.075};//0.02,0.02,0.02};//0.075,0.075};
float X_apo_height1[2] = {0.0f, 0.0f};
float P_apo_k_height1[4] = {100.0f,0.0f,0.0f,100.0f};
float r_baro1 = 10;//10; // 10.0f;			
float r_acc1 =  0.1; // 0.5f;
float k_spd=2;
#include "kf_oldx.h"
float gh=0.15;
float gh_sonar=0.001;
float ga=0.1;
float gwa=0.1;
double P_kf_baro[9]={1,0,0,1,0,0,1}; 
double X_kf_baro[3], X_kf_baro_temp[3];
double P_kf_sonar[9]={1,0,0,1,0,0,1}; 
double X_kf_sonar[3];
static void altDoPresUpdate(float measuredPres,float dt) {
    float noise;        // measurement variance
    float y;            // measurment
    float V[ALT_V];		// process variance
	  static u8 height_ctrl_moder;
	  float Q[ALT_S];		// state variance
		float ultra_sp_tmp,T,ultra_dis_tmp;
		static int ultra_distance_old;

    altUkfData_bmp.x = srcdkfGetState(altUkfData_bmp.kf);
	#if USE_UKF_SONAR
		altUkfData_sonar.x = srcdkfGetState(altUkfData_sonar.kf);
	#endif
	 y = (float)baroAlt/1000;
	   static u16 cnt_1;

	 #if defined(SONAR_SAMPLE1)
	 float temp_sonar;
	  y = (float)(Moving_Median(1,10,ultra_distance))/1000;
	 #elif defined(SONAR_SAMPLE2)
	  y = (float)(Moving_Median(1,5,ultra_distance))/1000;
	 #elif defined(SONAR_SAMPLE3)
	  y = (float)(Moving_Median(1,10,ultra_distance))/1000;
	 #endif
	  #if SONAR_USE_FLOW
    y=flow.hight.originf;
	  #endif
		#if USE_UKF_SONAR
		noise = ALT_PRES_NOISE_SONAR;
		Moving_Average( (float)( y),sonar_h_arr,SONAR_HIHG_NUM, sonar_h_cnt ,&sonar_temp);	 
		srcdkfMeasurementUpdate(altUkfData_sonar.kf, 0, &y, 1, 1, &noise, altUkfPresUpdate);
		ALT_POS_SONAR3=sonar_temp*K_SONAR/10+(1-K_SONAR/10)*(ALT_POS_SONAR3-T*ALT_VEL_BMP);
		ALT_POS_SONAR2=ALT_POS_SONAR;
		#else
		ultra_delta = ultra_distance - ultra_distance_old;
		ultra_distance_old = ultra_distance;
		sonar_filter((float)y,dt);
		static u8 init;
		if(!init){init=1;}
		ultra_sp_tmp=v_pred;
		ALT_VEL_SONAR=ultra_sp_tmp;
		Moving_Average( (float)( y),sonar_h_arr,SONAR_HIHG_NUM, sonar_h_cnt ,&sonar_temp);	 
		
	  //acc_fusion(T,&sonar_f_set,baro_matlab_data[1],&sonar_temp,&ALT_POS_SONAR3);
		
		
		uint8_t zFlag[2] = {1, 1};
		
	
		float oldx_sonar;
		#if SONAR_USE_FLOW
    //oldx_sonar=sonar_filter_oldx(flow.hight.originf);
		oldx_sonar=insert_sonar_value_and_get_mode_value(flow.hight.originf);
		#else
		//oldx_sonar=sonar_filter_oldx((float)ultra_distance/1000.);//<<----------------start
		oldx_sonar=insert_sonar_value_and_get_mode_value((float)ultra_distance/1000.);
	  #endif
		ultra.relative_height =oldx_sonar*100;
		fusion_prepare(dt,sonar_av_arr,SONAR_AV_NUM,&sonar_av_cnt,0,&ultra,&sonar);
		//acc_fusion(dt,&sonar_f_set,baro_matlab_data[1]*1000,&sonar,&sonar_fusion);
	  //float z[2] = { (float)sonar.displacement/1000., -(baro_matlab_data[1])};
		//HeightEKF( X_apo_height1, P_apo_k_height1,zFlag, dt, z, r_baro1, r_acc1, X_apo_height1, P_apo_k_height1);
   
		float temp=(reference_vr_imd_down[2] *imu_fushion.Acc.z + reference_vr_imd_down[0] *imu_fushion.Acc.x + reference_vr_imd_down[1] *imu_fushion.Acc.y - 4096  );
	  float acc_temp1=my_deathzoom(acc_neo[2],0.01);//my_deathzoom(((float)temp/4096.0f) *9.8,0.01);
	  //ALT_POS_SONAR3+=dt*ALT_VEL_BMP_EKF*k_spd;
		//double Z_kf[3]={(float)ultra_distance/1000,0,0};
		double Z_kf[3]={(float)oldx_sonar+X_kf_sonar[1]*T,0,0};
	  // kf_oldx( X_kf_sonar,  P_kf_sonar,  Z_kf,  acc_temp1, gh_sonar,  ga,  gwa,dt);
		
				
		 float ultra_dis_tmp1=oldx_sonar;//(float)sonar.displacement/1000;		
			if(amf.connect)
				ultra_dis_tmp1=amf.pos_o[2];
		if( fabs(ultra_dis_tmp1 - ALT_POS_SONAR3) < 0.1 )
		{
			
			ALT_POS_SONAR3 += ( 1 / ( 1 + 1 / ( K_SONAR*4.0f *3.14f *dt ) ) ) *(ultra_dis_tmp1 - ALT_POS_SONAR3) ;
		}
		else if( fabs(ultra_dis_tmp1 - ALT_POS_SONAR3) < 0.2 )
		{
			
			ALT_POS_SONAR3 += ( 1 / ( 1 + 1 / ( K_SONAR*2.2f *3.14f *dt ) ) ) *(ultra_dis_tmp1- ALT_POS_SONAR3) ;
		}
		else if( fabs(ultra_dis_tmp1 - ALT_POS_SONAR3) < 0.4 )
		{
			ALT_POS_SONAR3 += ( 1 / ( 1 + 1 / ( K_SONAR*1.2f *3.14f *dt ) ) ) *(ultra_dis_tmp1- ALT_POS_SONAR3) ;
		}
		else
		{
			ALT_POS_SONAR3 += ( 1 / ( 1 + 1 / ( K_SONAR*0.6f *3.14f *dt ) ) ) *(ultra_dis_tmp1- ALT_POS_SONAR3) ;
		}	
		
		
		if(sonar_fc!=0&&!ultra_ok)
		ALT_POS_SONAR2=sonar_fc;
		else
	  ALT_POS_SONAR2=ALT_POS_SONAR3;
	
			
		#endif
}



u8 OLDX_KF2(float *measure,float tau,float *r_sensor,u8 *flag_sensor,double *state,double *state_correct,float T)
{
float PosDealt;	
float SpeedDealt;
float K_ACC_Z;
float K_VEL_Z;
float K_POS_Z;

if(!flag_sensor[0]&&!flag_sensor[1]&&!flag_sensor[2])	
	return 0;
K_ACC_Z =(5.0f / (tau * tau * tau));
K_VEL_Z =(3.0f / (tau * tau));
K_POS_Z =(3.0f / tau);
//d spd	
if(flag_sensor[0]&&!flag_sensor[1])	
PosDealt=(measure[0]-state[0]);
else if(flag_sensor[0]&&!flag_sensor[1])
PosDealt=measure[1];
else if(flag_sensor[1]&&!flag_sensor[1])
PosDealt=(measure[0]-state[0])+state[1];
else 
return 0;	

state_correct[3*0+2] += r_sensor[0]*PosDealt* K_ACC_Z ;
state_correct[3*0+1] += r_sensor[1]*PosDealt* K_VEL_Z ;
state_correct[3*0+0] += r_sensor[2]*PosDealt* K_POS_Z ;

//acc correct
if(!flag_sensor[1]&&flag_sensor[2])	
state[2]=measure[2]+state_correct[0*3+2];
else if(flag_sensor[1]&&flag_sensor[2])	
state[2]=measure[1]+(measure[2]+state_correct[0*3+2]);
	
//d acc
SpeedDealt=state[2]*T;

//pos correct
state_correct[1*3+0]+=(state[1]+0.5*SpeedDealt)*T;
state[0]=state_correct[1*3+0]+state_correct[0*3+0];

//vel correct
state_correct[0*3+1]+=SpeedDealt;
state[1]=state_correct[1*3+1]+state_correct[0*3+1];

return 1;	
}


#define GPS_DELAY 0.0
static float acc_body_bufz[40];
static void feed_acc_bufz(float in1)
{
u8 i,j;	
float reg[40];	
static u8 cnt;
	for(j=0;j<40;j++)
   reg[j]=acc_body_bufz[j];
	

	for(j=0;j<40-1;j++)
   acc_body_bufz[j]=reg[j-1];	
	
	acc_body_bufz[0]=in1;

}	

static float get_acc_delayz(float delay,float dt)
{
u8 id[2];
id[0]=(int)(delay/dt);
id[1]=id[0]+1;	
if(delay>0)	
return acc_body_bufz[id[0]]/2+acc_body_bufz[id[1]]/2;
else
return acc_body_bufz[0];	
}	


#include "LIS3MDL.h"
float X_apo_height[2] = {0.0f, 0.0f};
float P_apo_k_height[4] = {100.0f,0.0f,0.0f,100.0f};
float k_bais=  0.0;
float k_bais2= 0;
float r_baro = 10;//10; // 10.0f;			
float r_acc =  0.1; // 0.5f;
				

float gro_z_dead=40,dead_accz=0.00020;
float acc_off_baro=0;
float k_hcc=0.0;
float k_hcc_fix=1;
float hc_acc_i_bmp;
float accz,accz_bmp;
float acc_z_view[3];
float acc_scale_bmp=1;
u8 acc_mask;
u16 acc_mask_cnt_max=1;
float baro_ukf_dt,baro_ekf_dt;
float k_flt_accz=0.75;
float acc_est,acc_est_imu;
u8 baro_ekf_ero;
float ALT_VEL_BMP_UNION,ALT_POS_BMP_UNION;
u8 baroAlt_sel=0;

float  r_baro_new[4]={0.015,0.05,0.03,3};
double state_correct_baro[6];

float g_baro_h=0.0025,g_baro_spd=0.0035;
void altUkfProcess(float measuredPres) {
static float wz_speed_old; 
float dt;
static int temp_r,temp;
float acc_temp1;  
float baroAlt_temp;
	  #if USE_M100_IMU
	  if(m100.connect&&m100.m100_data_refresh)
		baroAlt_temp=imu_fushion.Alt;
		else
		baroAlt_temp=baroAlt_fc;
	  #else
	  if(baroAlt_sel&&mpu6050.good)
	  baroAlt_temp=baroAlt;
		else
		baroAlt_temp=baroAlt_fc;
		#endif
		//baroAlt_temp=lis3mdl.Alt*1000;	
	  dt = Get_Cycle_T(GET_T_BARO_UKF);	
	
		acc_temp1=acc_neo[2]+ acc_off_baro;
	  baro_matlab_data[1]=accz_bmp=LIMIT(my_deathzoom(acc_temp1,dead_accz)*acc_scale_bmp,-3.6,3.6);
   
    if(!fly_ready||for_fly_ready)	{	
		if(!baro_ekf_ero)
		acc_off_baro=LIMIT((float)temp/4096.0f *9.8,-3,3);
	  X_apo_height[0] =baroAlt_temp/1000.;X_apo_height[1]=0;// {0.0f, 0.0f};
		P_apo_k_height[0] =P_apo_k_height[3]=100;P_apo_k_height[1]=P_apo_k_height[2]=0;// {100.0f,0.0f,0.0f,100.0f};
	  baro_ekf_ero=0;
		}
		#if USE_UKF_SONAR
		srcdkfTimeUpdate(altUkfData_sonar.kf, &acc_temp1,dt);//5000	
		#endif
    altDoPresUpdate(measuredPres,dt);
		wz_speed_old = ALT_VEL_BMP;
	  

		if(force_fly_ready!=0)fly_ready=1;
		baro_matlab_data[0]=(float)baroAlt_temp/1000.;
    static u8 cnt_ekf;
		 #define BARO_UKF
		 // #define BARO_KF2
		 // #define BARO_EKF

		if(mpu6050.good==0)
		baro_set=1;
		if(baro_set){cnt_ekf=0;
			
			#if defined(BARO_KF2) //UKF with limit bias
			u8 flag_sensor[3]={1,0,1};	
			float Z_kf[3]={(float)baroAlt_temp/1000.+LIMIT(my_deathzoom(X_kf_baro[1],0.68),-1,1)*dt*5,0,accz_bmp};
			OLDX_KF2(Z_kf,r_baro_new[3],r_baro_new,flag_sensor,X_kf_baro,state_correct_baro,dt);
			#elif defined(BARO_UKF) //UKF with limit bias
		 float z[2] = { (float)baroAlt_temp/1000., (baro_matlab_data[1])};
		 float noise = ALT_PRES_NOISE;	
		 srcdkfTimeUpdate(altUkfData_bmp.kf, &z[1],dt);//5000			    // us (200 Hz)		
     srcdkfMeasurementUpdate(altUkfData_bmp.kf, 0, &z[0], 1, 1, &noise, altUkfPresUpdate);
	   X_kf_baro[0]=altUkfData_bmp.x[ALT_STATE_POS]+X_kf_baro[1]*dt*10;
		 X_kf_baro[1]=-altUkfData_bmp.x[ALT_STATE_VEL];	
		 X_apo_height[0]= X_kf_baro[0];
		 X_apo_height[1]= X_kf_baro[1];
			#elif  defined(BARO_EKF) //KF with bias
			uint8_t zFlag[2] = {1, 1};
		  float z[2] = { (float)baroAlt_temp/1000., (accz_bmp)};//)+((float)baroAlt/1000.-X_apo_height[0]
      HeightEKF( X_apo_height, P_apo_k_height,zFlag, dt, z, r_baro, r_acc, X_apo_height, P_apo_k_height);
			X_kf_baro[0]=X_apo_height[0];
      X_kf_baro[1]=X_apo_height[1];
			#else
			feed_acc_bufz(accz_bmp);
			float T=dt;
			double A[9]=
			 {1,       0,    0,
				T,       1,    0,
				-T*T/2, -T,    1};

			double B[3]={T*T/2,T,0}; 
			double H[9]={
			 1,0,0,
			 0,1,0,
			 0,0,0}; 
      #if !USE_M100_IMU
			double Z_kf[3];	
			static float off_gps_baro; 
//			 if (gpsx.pvt.PVT_Hacc*0.001 < 0.8f&&gpsx.pvt.PVT_longitude!=0 && gpsx.pvt.PVT_numsv>=4&&gpsx.pvt.PVT_fixtype>=1)
//			 {
//			  Z_kf[0]=gpsx.pvt.PVT_height;
//				Z_kf[1]=gpsx.pvt.PVT_Down_speed; 
//				if(gpsx.pvt.PVT_height>0) 
//				off_gps_baro=gpsx.pvt.PVT_height-baroAlt_temp/1000.; 
//			 }
//			 else 
//			 {
				H[4]=0; 
			  Z_kf[0]= baroAlt_temp/1000.+off_gps_baro;
//			 }	 
		  #else
			double Z_kf[3]={m100.H,m100.spd[2],0};	
			#endif
			float accz_bmp_temp=get_acc_delayz(GPS_DELAY,T);
    	 //kf_oldx( X_kf_baro,  P_kf_baro,  Z_kf,  (accz_bmp), gh,  ga,  gwa,dt);
	
			KF_OLDX_NAV(X_kf_baro_temp,  P_kf_baro,  Z_kf,  accz_bmp_temp, A,  B,  H,  0.1,  0.1, g_baro_h,g_baro_spd,  T);
			X_kf_baro[0]=X_kf_baro_temp[0]+GPS_DELAY*X_kf_baro_temp[1]+1/2*pow(GPS_DELAY,2)*(accz_bmp_temp+0*X_kf_baro_temp[2]);
			X_kf_baro[1]=X_kf_baro_temp[1]+GPS_DELAY*(accz_bmp_temp+0*X_kf_baro_temp[2]);
			X_kf_baro[2]=X_kf_baro_temp[2];
			
      #endif
			
		}
		
	static float spd_r_imu;
  acc_est_imu=(X_apo_height[1]-spd_r_imu)/dt;
	spd_r_imu=X_apo_height[1];
	if(fabs(acc_est_imu)>5||fabs(spd_r_imu)>10)	
  {baro_ekf_ero=1;}
	
	//if(fabs(X_kf_sonar[1])<0.1&&fabs(X_kf_baro[1])<0.1&&ALT_POS_SONAR3<2.6&&ultra_ok)
	//ALT_VEL_BMP_UNION=X_kf_sonar[1];	
	//else
	ALT_VEL_BMP_UNION=X_kf_baro[1];
}

int acc_flag[2]={1,1};
float k_acc_ukf=0.2;//20;
float MAX_ACC_NEO=0.8;
float flow_dt;
void FlowUkfProcess(float measuredPres) {

static   float wz;
float accz_x,accz_y;  
flow_dt = Get_Cycle_T(GET_T_FLOW_UKF);	
  
accz_x =flow_matlab_data[0];//k_acc_ukf*accz_x +(1-k_acc_ukf)*(acc_neo[0]*acc_flag[0]);
accz_y =flow_matlab_data[1];//k_acc_ukf*accz_y +(1-k_acc_ukf)*(acc_neo[1]*acc_flag[1]);
accz_x=LIMIT(accz_x,-MAX_ACC_NEO,MAX_ACC_NEO);
accz_y=LIMIT(accz_y,-MAX_ACC_NEO,MAX_ACC_NEO);
static u16 cnt_init;
	if(cnt_init++>100){cnt_init=101;
    srcdkfTimeUpdate(flowUkfData_x.kf, &accz_x,flow_dt);//5000			    // us (200 Hz)
    srcdkfTimeUpdate(flowUkfData_y.kf, &accz_y,flow_dt);//5000			    // us (200 Hz)
	  flow_ukf_Update(flow_dt);
	}
    
}

GPS_Sensor_Struct Global_GPS_Sensor;

//flow
float GPS_NOISE =0.00052f;
//0--lad N 1--lon E
double v_test[2],Last_GPS[2],TAR_GPS[2];
static void gps_ukf_Update(float dt) {
    float noise;        // measurement variance
    float y[2];            // measurment
    gpsUkfData_n.x = srcdkfGetState(gpsUkfData_n.kf);
		gpsUkfData_e.x = srcdkfGetState(gpsUkfData_e.kf);

static u16 init_cnt,cnt_1m;
	if(init_cnt++>100){init_cnt=101;

		
	if(cnt_1m++>0.11/dt){cnt_1m=0;	
	v_test[0]=(Global_GPS_Sensor.NED_Pos[0]-Last_GPS[0])/0.11	;
	v_test[1]=(Global_GPS_Sensor.NED_Pos[1]-Last_GPS[1])/0.11	;
	Last_GPS[0]=Global_GPS_Sensor.NED_Pos[0];
	Last_GPS[1]=Global_GPS_Sensor.NED_Pos[1];
	}
	y[0] =Global_GPS_Sensor.NED_Vel[0];y[1] =Global_GPS_Sensor.NED_Vel[1];
	if(fabs(Global_GPS_Sensor.NED_Pos[0]-TAR_GPS[0])>1000||fabs(Global_GPS_Sensor.NED_Pos[1]-TAR_GPS[1])>1000)
	{
		TAR_GPS[0]=Global_GPS_Sensor.NED_Pos[0];
	  TAR_GPS[1]=Global_GPS_Sensor.NED_Pos[1];
	}
	y[0] =LIMIT(Global_GPS_Sensor.NED_Vel[0],-100,100);y[1] =LIMIT(Global_GPS_Sensor.NED_Vel[1],-100,100);
	
	noise = GPS_NOISE;
	srcdkfMeasurementUpdate(gpsUkfData_n.kf, 0, &y[0], 1, 1, &noise, gpsUpdate);
	srcdkfMeasurementUpdate(gpsUkfData_e.kf, 0, &y[1], 1, 1, &noise, gpsUpdate);
	}
}
/*
1 n
||_____0 e
*/
float k_acc_fpl_gps=0.75;
void GpsUkfProcess(float T) {

static   float wz;
float accz_e,accz_n;
static float acc_temp[2];  

//GPS_Dates_Deal();
acc_temp[0] =k_acc_fpl_gps*acc_temp[0]  +(1-k_acc_fpl_gps)*(acc_neo[0]);
acc_temp[1] =k_acc_fpl_gps*acc_temp[1]  +(1-k_acc_fpl_gps)*(acc_neo[1]);
accz_n=cos(Yaw*0.017)*(-acc_temp[1])-	sin(Yaw*0.017)*(-acc_temp[0]);
accz_e=cos(Yaw*0.017)*(-acc_temp[0])+ sin(Yaw*0.017)*(-acc_temp[1]);	

//	
		
acc_z_view[0]=accz_n*1000;
acc_z_view[1]=accz_e*1000;
accz_e=LIMIT(accz_e,-MAX_ACC_NEO,MAX_ACC_NEO);
accz_n=LIMIT(accz_n,-MAX_ACC_NEO,MAX_ACC_NEO);
static u16 cnt_init;
	if(cnt_init++>100){cnt_init=101;
    srcdkfTimeUpdate(gpsUkfData_n.kf, &accz_n,T);//5000			    // us (200 Hz)
    srcdkfTimeUpdate(gpsUkfData_e.kf, &accz_e,T);//5000			    // us (200 Hz)
	  gps_ukf_Update(T);
	}
    
}

//alt
float q_pos = 5.0f;
float v_q=1e-6f;
float v_bais=0.0005;//0.0125;//0.0125;//0.0125;//0.0005;
float s_bais=0.05f;

//flow
float ALT_STATE_POS1=5;
float ALT_STATE_VEL1= 1e-6f;
float ALT_STATE_BIAS1=0.05f;
float ALT_NOISE_BIAS1=0.0005;
float ALT_NOISE_VEL1=0.0005;

//GPS
float ALT_STATE_POS_GPS=5;
float ALT_STATE_VEL_GPS= 1e-6f;
float ALT_STATE_BIAS_GPS=0.05f;
float ALT_NOISE_BIAS_GPS=0.0005;
float ALT_NOISE_VEL_GPS=0.0005;
void altUkfInit(void){
    float Q[ALT_S];		// state variance
    float V[ALT_V];		// process variance
		float Q_flow[ALT_S];		// state variance
    float V_flow[ALT_V];		// process variance
	  float Q_gps[ALT_S];		// state variance
    float V_gps[ALT_V];		// process variance
    altUkfData_bmp.kf = srcdkfInit(ALT_S, ALT_M, ALT_V, ALT_N, altUkfTimeUpdate);
    altUkfData_bmp.x = srcdkfGetState(altUkfData_bmp.kf);
	  #if USE_UKF_SONAR	
		altUkfData_sonar.kf = srcdkfInit(ALT_S, ALT_M, ALT_V, ALT_N, altUkfTimeUpdate);
    altUkfData_sonar.x = srcdkfGetState(altUkfData_sonar.kf);
    #endif
    Q[ALT_STATE_POS] = q_pos;
    Q[ALT_STATE_VEL] = v_q;
    Q[ALT_STATE_BIAS] = s_bais;//0.05f;
		V[ALT_NOISE_BIAS] = 5e-4f;
	  V[ALT_NOISE_VEL]=  5e-4f;
    srcdkfSetVariance(altUkfData_bmp.kf, Q, V, 0, 0);
		#if USE_UKF_SONAR	
	  Q[ALT_STATE_POS] = 5.0f;
    Q[ALT_STATE_VEL] = 1e-6f;
    Q[ALT_STATE_BIAS] = 0.05f;
		V[ALT_NOISE_BIAS] =  0.0025;//0.5;
		V[ALT_NOISE_VEL] =  0.0025;//0.5;
    srcdkfSetVariance(altUkfData_sonar.kf, Q, V, 0, 0);
		#endif

		ALT_POS_BMP =0;
		ALT_VEL_BMP  =0;
		ALT_BIAS_BMP =0;
		ALT_POS_SONAR =0;
		ALT_VEL_SONAR  =0;
		ALT_BIAS_SONAR =0;
	
//--------------------------------------------------
    flowUkfData_x.kf = srcdkfInit(ALT_S, ALT_M, ALT_V, ALT_N, flowUkfTimeUpdate);
    flowUkfData_x.x = srcdkfGetState(flowUkfData_x.kf);
		flowUkfData_y.kf = srcdkfInit(ALT_S, ALT_M, ALT_V, ALT_N, flowUkfTimeUpdate);
    flowUkfData_y.x = srcdkfGetState(flowUkfData_y.kf);

    Q_flow[ALT_STATE_POS] = ALT_STATE_POS_GPS;
    Q_flow[ALT_STATE_VEL] = ALT_STATE_VEL_GPS;
    Q_flow[ALT_STATE_BIAS]= ALT_STATE_BIAS_GPS;
		V_flow[ALT_NOISE_BIAS]= ALT_NOISE_BIAS_GPS;
	  V_flow[ALT_NOISE_VEL] = ALT_NOISE_VEL_GPS;

		srcdkfSetVariance(flowUkfData_x.kf, Q_flow, V_flow, 0, 0);
		srcdkfSetVariance(flowUkfData_y.kf, Q_flow, V_flow, 0, 0);

		FLOW_POS_X =0;
		FLOW_VEL_X  =0;
		FLOW_BIAS_X =0;
		FLOW_POS_X =0;
		FLOW_VEL_X  =0;
		FLOW_BIAS_Y =0;


//-------------------------------GPS----------------
    gpsUkfData_n.kf = srcdkfInit(ALT_S, ALT_M, ALT_V, ALT_N, altUkfTimeUpdate);
    gpsUkfData_n.x = srcdkfGetState(gpsUkfData_n.kf);
	  gpsUkfData_e.kf = srcdkfInit(ALT_S, ALT_M, ALT_V, ALT_N, altUkfTimeUpdate);
    gpsUkfData_e.x = srcdkfGetState(gpsUkfData_e.kf);
    Q_gps[ALT_STATE_POS] = ALT_STATE_POS_GPS;
    Q_gps[ALT_STATE_VEL] = ALT_STATE_VEL_GPS;
    Q_gps[ALT_STATE_BIAS] = ALT_STATE_BIAS_GPS;
		V_gps[ALT_NOISE_BIAS] = ALT_NOISE_BIAS_GPS;
	  V_gps[ALT_NOISE_VEL] =ALT_NOISE_VEL_GPS;
    srcdkfSetVariance(gpsUkfData_e.kf, Q_gps, V_gps, 0, 0);
		srcdkfSetVariance(gpsUkfData_n.kf, Q_gps, V_gps, 0, 0);
}

