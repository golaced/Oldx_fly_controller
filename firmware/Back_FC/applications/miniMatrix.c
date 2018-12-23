#ifndef _MINIMAXTRIX_H_
#define _MINIMAXTRIX_H_

#include "miniMatrix.h"

//////////////////////////////////////////////////////////////////////////
//
void Matrix_Zero(float *A, unsigned short numRows, unsigned short numCols)
{
	float *pIn = A;
	unsigned int numSamples = numRows * numCols;
	unsigned int blkCnt = numSamples >> 2u;

	//cortex-m3's speed optimization
	while(blkCnt > 0u){
		(*pIn++) = 0.0f;
		(*pIn++) = 0.0f;
		(*pIn++) = 0.0f;
		(*pIn++) = 0.0f;
		blkCnt--;
	}
	blkCnt = numSamples & 0x03u;

	while(blkCnt > 0u){
		(*pIn++) = 0.0f;
		blkCnt--;
	}
}

void Matrix_Copy(float *pSrc, unsigned short numRows, unsigned short numCols, float *pDst)
{	
	unsigned int numSamples; // total number of elements in the matrix 
	unsigned int blkCnt; // loop counters

	float in1, in2, in3, in4;

	// Total number of samples in the input matrix
	numSamples = (unsigned int) numRows * numCols;

	// Loop unrolling
	blkCnt = numSamples >> 2u;
	// First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
	// a second loop below computes the remaining 1 to 3 samples.
	while(blkCnt > 0u){
		// C = A
		// Copy and then store the results in the destination buffer
		in1 = *pSrc++;
		in2 = *pSrc++;
		in3 = *pSrc++;
		in4 = *pSrc++;

		*pDst++ = in1;
		*pDst++ = in2;
		*pDst++ = in3;
		*pDst++ = in4;

		// Decrement the loop counter
		blkCnt--;
	}

	// If the numSamples is not a multiple of 4, compute any remaining output samples here.    
	// No loop unrolling is used.
	blkCnt = numSamples & 0x3u;

	while(blkCnt > 0u){
		// C = A
		// Copy and then store the results in the destination buffer
		*pDst++ = *pSrc++;

		// Decrement the loop counter
		blkCnt--;
	}
}

int Maxtrix_Add(float *pSrcA, unsigned short numRows, unsigned short numCols, float *pSrcB, float *pDst)
{
	float *pIn1 = pSrcA; // input data matrix pointer A 
	float *pIn2 = pSrcB; // input data matrix pointer B 
	float *pOut = pDst; // output data matrix pointer  
	float inA1, inA2, inB1, inB2, out1, out2; // temporary variables

	unsigned int numSamples; // total number of elements in the matrix 
	unsigned int blkCnt; // loop counters
	int status; // status of matrix addition

	// Total number of samples in the input matrix
	numSamples = (unsigned int) numRows * numCols;

	// Loop unrolling
	blkCnt = numSamples >> 2u;

	// First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
	// a second loop below computes the remaining 1 to 3 samples.
	while(blkCnt > 0u){
		// C(m,n) = A(m,n) + B(m,n)
		// Add and then store the results in the destination buffer.
		// Read values from source A
		inA1 = pIn1[0];

		// Read values from source B
		inB1 = pIn2[0];

		// Read values from source A
		inA2 = pIn1[1];

		// out = sourceA + sourceB
		out1 = inA1 + inB1;

		// Read values from source B
		inB2 = pIn2[1];

		// Read values from source A
		inA1 = pIn1[2];

		// out = sourceA + sourceB
		out2 = inA2 + inB2;

		// Read values from source B
		inB1 = pIn2[2];

		// Store result in destination
		pOut[0] = out1;
		pOut[1] = out2;

		// Read values from source A
		inA2 = pIn1[3];

		// Read values from source B
		inB2 = pIn2[3];

		// out = sourceA + sourceB
		out1 = inA1 + inB1;

		// out = sourceA + sourceB
		out2 = inA2 + inB2;

		// Store result in destination
		pOut[2] = out1;

		// Store result in destination
		pOut[3] = out2;


		// update pointers to process next sampels
		pIn1 += 4u;
		pIn2 += 4u;
		pOut += 4u;
		// Decrement the loop counter
		blkCnt--;
	}

	// If the numSamples is not a multiple of 4, compute any remaining output samples here.    
	// No loop unrolling is used.
	blkCnt = numSamples & 0x3u;


	while(blkCnt > 0u){
		// C(m,n) = A(m,n) + B(m,n)
		// Add and then store the results in the destination buffer.
		*pOut++ = (*pIn1++) + (*pIn2++);

		// Decrement the loop counter
		blkCnt--;
	}

	// set status as SUCCESS
	status = 0;

	// Return to application
	return (status);
}

int Maxtrix_Sub(float *pSrcA, unsigned short numRows, unsigned short numCols, float *pSrcB, float *pDst)
{
	float *pIn1 = pSrcA;                // input data matrix pointer A
	float *pIn2 = pSrcB;                // input data matrix pointer B
	float *pOut = pDst;                 // output data matrix pointer 

	float inA1, inA2, inB1, inB2, out1, out2;  // temporary variables

	unsigned int numSamples;                           // total number of elements in the matrix 
	unsigned int blkCnt;                               // loop counters
	int status;                             // status of matrix subtraction

	// Total number of samples in the input matrix
	numSamples = (unsigned int) numRows * numCols;

	// Run the below code for Cortex-M4 and Cortex-M3

	// Loop Unrolling
	blkCnt = numSamples >> 2u;

	// First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
	// a second loop below computes the remaining 1 to 3 samples.
	while(blkCnt > 0u){
		// C(m,n) = A(m,n) - B(m,n)
		// Subtract and then store the results in the destination buffer.
		// Read values from source A
		inA1 = pIn1[0];

		// Read values from source B
		inB1 = pIn2[0];

		// Read values from source A
		inA2 = pIn1[1];

		// out = sourceA - sourceB
		out1 = inA1 - inB1;

		// Read values from source B
		inB2 = pIn2[1];

		// Read values from source A
		inA1 = pIn1[2];

		// out = sourceA - sourceB
		out2 = inA2 - inB2;

		// Read values from source B
		inB1 = pIn2[2];

		// Store result in destination
		pOut[0] = out1;
		pOut[1] = out2;

		// Read values from source A
		inA2 = pIn1[3];

		// Read values from source B
		inB2 = pIn2[3];

		// out = sourceA - sourceB
		out1 = inA1 - inB1;

		// out = sourceA - sourceB
		out2 = inA2 - inB2;

		// Store result in destination
		pOut[2] = out1;

		// Store result in destination
		pOut[3] = out2;

		// update pointers to process next sampels
		pIn1 += 4u;
		pIn2 += 4u;
		pOut += 4u;

		// Decrement the loop counter
		blkCnt--;
	}

	// If the numSamples is not a multiple of 4, compute any remaining output samples here.    
	// No loop unrolling is used.
	blkCnt = numSamples & 0x3u;

	while(blkCnt > 0u){
		// C(m,n) = A(m,n) - B(m,n)
		// Subtract and then store the results in the destination buffer.
		*pOut++ = (*pIn1++) - (*pIn2++);

		// Decrement the loop counter
		blkCnt--;
	}

	// Set status as ARM_MATH_SUCCESS
	status = 0;

	// Return to application
	return (status);
}

int Matrix_Multiply(float* pSrcA, unsigned short numRowsA, unsigned short numColsA, float* pSrcB, unsigned short numColsB, float* pDst)
{
	float *pIn1 = pSrcA; // input data matrix pointer A
	float *pIn2 = pSrcB; // input data matrix pointer B
	float *pInA = pSrcA; // input data matrix pointer A 
	float *pOut = pDst; // output data matrix pointer
	float *px; // Temporary output data matrix pointer
	float sum; // Accumulator

	// Run the below code for Cortex-M4 and Cortex-M3

	float in1, in2, in3, in4;
	unsigned short col, i = 0u, j, row = numRowsA, colCnt; // loop counters
	int status; // status of matrix multiplication

	// The following loop performs the dot-product of each row in pSrcA with each column in pSrcB
	// row loop
	do{
		// Output pointer is set to starting address of the row being processed
		px = pOut + i;

		// For every row wise process, the column loop counter is to be initiated
		col = numColsB;

		// For every row wise process, the pIn2 pointer is set    
		// to the starting address of the pSrcB data
		pIn2 = pSrcB;

		j = 0u;

		// column loop
		do{
			// Set the variable sum, that acts as accumulator, to zero
			sum = 0.0f;

			// Initiate the pointer pIn1 to point to the starting address of the column being processed
			pIn1 = pInA;

			// Apply loop unrolling and compute 4 MACs simultaneously.
			colCnt = numColsA >> 2u;

			// matrix multiplication       
			while(colCnt > 0u){
				// c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n)
				in3 = *pIn2;
				pIn2 += numColsB;
				in1 = pIn1[0];
				in2 = pIn1[1];
				sum += in1 * in3;
				in4 = *pIn2;
				pIn2 += numColsB;
				sum += in2 * in4;

				in3 = *pIn2;
				pIn2 += numColsB;
				in1 = pIn1[2];
				in2 = pIn1[3];
				sum += in1 * in3;
				in4 = *pIn2;
				pIn2 += numColsB;
				sum += in2 * in4;
				pIn1 += 4u;

				// Decrement the loop count
				colCnt--;
			}

			// If the columns of pSrcA is not a multiple of 4, compute any remaining MACs here.    
			// No loop unrolling is used.
			colCnt = numColsA & 0x3u;

			while(colCnt > 0u){
				// c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n)
				sum += *pIn1++ * (*pIn2);
				pIn2 += numColsB;

				// Decrement the loop counter
				colCnt--;
			}

			// Store the result in the destination buffer
			*px++ = sum;

			// Update the pointer pIn2 to point to the  starting address of the next column
			j++;
			pIn2 = pSrcB + j;

			// Decrement the column loop counter
			col--;

		} while(col > 0u);

		// Update the pointer pInA to point to the  starting address of the next row
		i = i + numColsB;
		pInA = pInA + numColsA;

		// Decrement the row loop counter
		row--;

	} while(row > 0u);
	// Set status as ARM_MATH_SUCCESS
	status = 0;

	// Return to application
	return (status);
}

void Matrix_Multiply_With_Transpose(float *A, unsigned short nrows, unsigned short ncols, float *B, unsigned short mrows, float *C) 
{
	int i,j,k;
	float *pA;
	float *pB;

	for (i = 0; i < nrows; A += ncols, i++){
		for (pB = B, j = 0; j < mrows; C++, j++){
			for (pA = A, *C = 0.0, k  = 0; k < ncols; k++){
				*C += *pA++ * *pB++;
			}
		}
	}
}

void Maxtrix_Transpose(float *pSrc, unsigned short nRows, unsigned short nCols, float *pDst)
{
	float *pIn = pSrc;
	float *pOut = pDst;
	float *px;

	unsigned short blkCnt, i = 0u, row = nRows;

	do{
		blkCnt = nCols >> 2;
		px = pOut + i;

		while(blkCnt > 0u){
			*px = *pIn++;
			px += nRows;

			*px = *pIn++;
			px += nRows;

			*px = *pIn++;
			px += nRows;

			*px = *pIn++;
			px += nRows;

			blkCnt--;
		}
		blkCnt = nCols & 0x03u;
		while(blkCnt > 0u){
			*px = *pIn++;
			px += nRows;
			blkCnt--;
		}

		i++;
		row--;
	} while(row > 0u);
}

int Matrix_Inverse(float * pSrc, unsigned short n, float* pDst)
{
	float *pIn = pSrc; // input data matrix pointer
	float *pOut = pDst; // output data matrix pointer
	float *pInT1, *pInT2; // Temporary input data matrix pointer
	float *pOutT1, *pOutT2; // Temporary output data matrix pointer
	float *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst; // Temporary input and output data matrix pointer
	float maxC; // maximum value in the column

	float Xchg, in = 0.0f, in1; // Temporary input values 
	unsigned int i, rowCnt, flag = 0u, j, loopCnt, k, l; // loop counters
	int status; // status of matrix inverse

	// Working pointer for destination matrix
	pOutT1 = pOut;

	// Loop over the number of rows
	rowCnt = n;

	// Making the destination matrix as identity matrix
	while(rowCnt > 0u){
		// Writing all zeroes in lower triangle of the destination matrix
		j = n - rowCnt;
		while(j > 0u){
			*pOutT1++ = 0.0f;
			j--;
		}

		// Writing all ones in the diagonal of the destination matrix
		*pOutT1++ = 1.0f;

		// Writing all zeroes in upper triangle of the destination matrix
		j = rowCnt - 1u;
		while(j > 0u){
			*pOutT1++ = 0.0f;
			j--;
		}

		// Decrement the loop counter
		rowCnt--;
	}

	// Loop over the number of columns of the input matrix.    
	// All the elements in each column are processed by the row operations
	loopCnt = n;

	// Index modifier to navigate through the columns
	l = 0u;

	while(loopCnt > 0u){
		// Check if the pivot element is zero..    
		// If it is zero then interchange the row with non zero row below.    
		// If there is no non zero element to replace in the rows below,    
		// then the matrix is Singular.

		// Working pointer for the input matrix that points    
		// to the pivot element of the particular row 
		pInT1 = pIn + (l * n);

		// Working pointer for the destination matrix that points    
		// to the pivot element of the particular row 
		pOutT1 = pOut + (l * n);

		// Temporary variable to hold the pivot value
		in = *pInT1;

		// Grab the most significant value from column l
		maxC = 0;
		for (i = l; i < n; i++){
			maxC = *pInT1 > 0 ? (*pInT1 > maxC ? *pInT1 : maxC) : (-*pInT1 > maxC ? -*pInT1 : maxC);
			pInT1 += n;
		}

		// Update the status if the matrix is singular
		if(maxC == 0.0f){
			return -1;
		}

		// Restore pInT1 
		pInT1 = pIn;

		// Destination pointer modifier
		k = 1u;

		// Check if the pivot element is the most significant of the column
		if( (in > 0.0f ? in : -in) != maxC){
			// Loop over the number rows present below
			i = n - (l + 1u);

			while(i > 0u){
				// Update the input and destination pointers
				pInT2 = pInT1 + (n * l);
				pOutT2 = pOutT1 + (n * k);

				// Look for the most significant element to    
				// replace in the rows below
				if((*pInT2 > 0.0f ? *pInT2: -*pInT2) == maxC){
					// Loop over number of columns    
					// to the right of the pilot element
					j = n - l;

					while(j > 0u){
						// Exchange the row elements of the input matrix
						Xchg = *pInT2;
						*pInT2++ = *pInT1;
						*pInT1++ = Xchg;

						// Decrement the loop counter
						j--;
					}

					// Loop over number of columns of the destination matrix
					j = n;

					while(j > 0u){
						// Exchange the row elements of the destination matrix
						Xchg = *pOutT2;
						*pOutT2++ = *pOutT1;
						*pOutT1++ = Xchg;

						// Decrement the loop counter
						j--;
					}

					// Flag to indicate whether exchange is done or not
					flag = 1u;

					// Break after exchange is done
					break;
				}

				// Update the destination pointer modifier
				k++;

				// Decrement the loop counter
				i--;
			}
		}

		// Update the status if the matrix is singular
		if((flag != 1u) && (in == 0.0f)){
			return -1;
		}

		// Points to the pivot row of input and destination matrices
		pPivotRowIn = pIn + (l * n);
		pPivotRowDst = pOut + (l * n);

		// Temporary pointers to the pivot row pointers
		pInT1 = pPivotRowIn;
		pInT2 = pPivotRowDst;

		// Pivot element of the row
		in = *pPivotRowIn;

		// Loop over number of columns    
		// to the right of the pilot element
		j = (n - l);

		while(j > 0u){
			// Divide each element of the row of the input matrix    
			// by the pivot element
			in1 = *pInT1;
			*pInT1++ = in1 / in;

			// Decrement the loop counter
			j--;
		}

		// Loop over number of columns of the destination matrix
		j = n;

		while(j > 0u){
			// Divide each element of the row of the destination matrix    
			// by the pivot element
			in1 = *pInT2;
			*pInT2++ = in1 / in;

			// Decrement the loop counter
			j--;
		}

		// Replace the rows with the sum of that row and a multiple of row i    
		// so that each new element in column i above row i is zero.*/

		// Temporary pointers for input and destination matrices
		pInT1 = pIn;
		pInT2 = pOut;

		// index used to check for pivot element
		i = 0u;

		// Loop over number of rows
		//  to be replaced by the sum of that row and a multiple of row i
		k = n;

		while(k > 0u){
			// Check for the pivot element
			if(i == l){
				// If the processing element is the pivot element,    
				// only the columns to the right are to be processed
				pInT1 += n - l;

				pInT2 += n;
			}
			else{
				// Element of the reference row
				in = *pInT1;

				// Working pointers for input and destination pivot rows
				pPRT_in = pPivotRowIn;
				pPRT_pDst = pPivotRowDst;

				// Loop over the number of columns to the right of the pivot element,    
				// to replace the elements in the input matrix
				j = (n - l);

				while(j > 0u){
					// Replace the element by the sum of that row    
					// and a multiple of the reference row 
					in1 = *pInT1;
					*pInT1++ = in1 - (in * *pPRT_in++);

					// Decrement the loop counter
					j--;
				}

				// Loop over the number of columns to    
				// replace the elements in the destination matrix
				j = n;

				while(j > 0u){
					// Replace the element by the sum of that row    
					// and a multiple of the reference row 
					in1 = *pInT2;
					*pInT2++ = in1 - (in * *pPRT_pDst++);

					// Decrement the loop counter
					j--;
				}

			}

			// Increment the temporary input pointer
			pInT1 = pInT1 + l;

			// Decrement the loop counter
			k--;

			// Increment the pivot index
			i++;
		}

		// Increment the input pointer
		pIn++;

		// Decrement the loop counter
		loopCnt--;

		// Increment the index modifier
		l++;
	}

	// Set status as SUCCESS
	status = 0;

	if((flag != 1u) && (in == 0.0f)){
		status = -1;
	}

	// Return to application
	return (status);
}

#endif
