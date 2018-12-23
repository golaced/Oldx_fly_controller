#ifndef MATLIB_H
#define MATLIB_H
/* print array */
void print(float *arr, int m, int n);

/* Matrix Multiplication */
void MatMultiplication(const float *matA, const unsigned int sizeA[], const float *matB, const unsigned int sizeB[], float *matC);

/* Determinant of 3by3 Matrix */
float MatDet3by3(const float *matA);

/*  matrix addition, mat A and B as inputs and C = A + B */
void MatAddition(const float *matA, const unsigned int sizeA[], const float *matB, float *matC);
void MatSubAddition(const float *matA, const unsigned int sizeA[], const float *matB, float *matC);
/* Transpose of mat A must be square */
void MatTranspose(const float *matA, const unsigned int sizeA[], float *matB);

/* 3 X 3 matrix inverse */
void MatInv3by3(const float *matA, float *matB);

/* 2 X 2 matrix determinant */
float MatDet2by2(const float *matA);

/* Matrix multiplied by scalar must be of same size*/
void MatScalarMult(const float *matA, const unsigned int sizeA[],const float scalar, float *matB);

/* 2X2 matrix inverse */
void MatInv2by2(const float *matA, float *matB);

/* Norm of Matrix */
float NormofMatrix(const float *matA, const unsigned int sizeA[]);

/*Normalize Vector */
void NormalizeMatrix(const float *matA, const unsigned int sizeA[], float *matB);

/*Calculate Cross Skew Matrix */
void crossSkew(const float *matA, float *matB);
#endif 
