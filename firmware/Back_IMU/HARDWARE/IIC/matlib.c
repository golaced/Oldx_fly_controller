#include <stdio.h>
#include <math.h>

/* print array */
void print(float *arr, int m, int n)
{
    int i, j;
    for (i = 0; i < m; i++){
      for (j = 0; j < n; j++)
       ;// printf("%f ", *((arr+i*n) + j));
      ;//printf("\n");
    }	

}


/* Matrix Multiplication */
void MatMultiplication(const float *matA, const unsigned int sizeA[], const float *matB, const unsigned int sizeB[], float *matC){
	unsigned int i1, j1, j2;
	for (i1 = 0 ; i1< sizeA[0]; i1++){
		for (j2 = 0; j2 < sizeB[1]; j2++){
			for (j1 = 0; j1 < sizeA[1] ; j1++){
					//printf("%d %d %d %d = %d %d \n", i1, j1, j1, j2, i1, j2);
					*((matC+i1*sizeB[1]) + j2) += (*((matA+i1*sizeA[1]) + j1)) * (*((matB+j1*sizeB[1]) + j2));
					//matC[i1][j2] += matA[i1][j1]*matB[i2][j2];
			}			
		}
	}
}


/* Determinant of 3by3 Matrix */
float MatDet3by3(const float *matA) {
	  return (*(matA)) * (((*((matA+1*3) + 1)) * (*((matA+2*3) + 2)) - (*((matA+1*3) + 2)) * (*((matA+2*3) + 1))))
	   -  (*((matA) + 1)) * ((*((matA+1*3))) * (*((matA+2*3) + 2)) - (*((matA+1*3) + 2)) * (*((matA+2*3))))
	   +  (*((matA) + 2)) * ((*((matA+1*3))) * (*((matA+2*3) + 1)) - (*((matA+1*3) + 1)) * (*((matA+2*3)))) ; 
}


/*  matrix addition, mat A and B as inputs and C = A + B */
void MatAddition(const float *matA, const unsigned int sizeA[], const float *matB, float *matC){
        unsigned int i1, j1;
        for (i1 = 0 ; i1< sizeA[0]; i1++){
                for (j1 = 0; j1 < sizeA[1]; j1++){
                         *((matC+i1*sizeA[1]) + j1) = (*((matA+i1*sizeA[1]) + j1)) + *((matB+i1*sizeA[1]) + j1);
                }
        }
}

/*  matrix addition, mat A and B as inputs and C = A + B */
void MatSubAddition(const float *matA, const unsigned int sizeA[], const float *matB, float *matC){
        unsigned int i1, j1;
        for (i1 = 0 ; i1< sizeA[0]; i1++){
                for (j1 = 0; j1 < sizeA[1]; j1++){
                         *((matC+i1*sizeA[1]) + j1) = (*((matA+i1*sizeA[1]) + j1)) - *((matB+i1*sizeA[1]) + j1);
                }
        }
}

/* Transpose of mat A must be square */
void MatTranspose(const float *matA, const unsigned int sizeA[], float *matB){
        unsigned int i1, j1;
        //print((float *)matA, 3, 3);
        for (i1 = 0 ; i1< sizeA[0]; i1++){
                for (j1 = 0; j1 < sizeA[1]; j1++){
                         *((matB+i1*sizeA[1]) + j1) = (*((matA+j1*sizeA[1]) + i1)) ;
                }
        }
}


/* 3 X 3 matrix inverse */

void MatInv3by3(const float *matA, float *matB){

	float det = MatDet3by3( (float *)matA);

	*(matB) = (-(*((matA+2*3) + 1)) * (*((matA+1*3) + 2)) + (*((matA+1*3) + 1)) * (*((matA+2*3) + 2))) / det;

	(*(matB + 1)) = (-(*((matA) + 1)) * (*((matA+2*3) + 2)) + (*((matA) + 2)) * (*((matA+2*3) + 1))) / det;

	(*(matB + 2)) = ((*((matA) + 1)) * (*((matA+1*3) + 2)) - (*((matA) + 2)) * (*((matA+1*3) + 1))) / det;



	(*(matB+3)) = ((*((matA+2*3))) * (*((matA+3) + 2)) - (*((matA+3))) * (*((matA+2*3) + 2))) / det;

	(*((matB+3) + 1)) = (-(*((matA+2*3))) * (*((matA) + 2)) + (*((matA)) * (*((matA+2*3) + 2))))/ det;

	(*((matB+3) + 2)) = ((*((matA+3))) * (*((matA) + 2)) - (*((matA)) * (*((matA+1*3) + 2)))) / det;



	(*(matB+2*3)) = (-(*((matA+2*3))) * (*((matA+3) + 1)) + (*((matA+3))) * (*((matA+2*3) + 1))) / det;

	(*((matB+2*3) + 1)) = ((*((matA+2*3))) * (*((matA) + 1)) - (*((matA))) * (*((matA+2*3) + 1))) / det;

	(*((matB+2*3) + 2)) = (-(*((matA+1*3))) * (*((matA) + 1)) + (*((matA))) * (*((matA+3) + 1))) / det;

}

/* 2 X 2 matrix determinant */

float MatDet2by2(const float *matA) {

  return ((*(matA)) * (*((matA+2)+1)) - (*((matA)+1)) * (*(matA+2)));

}


/* Matrix multiplied by scalar must be of same size*/

void MatScalarMult(const float *matA, const unsigned int sizeA[],const float scalar, float *matB){
        unsigned int i, j;
        for (i = 0 ; i< sizeA[0]; i++){
                for (j = 0; j < sizeA[1]; j++){
                         *((matB+i*sizeA[1]) + j) = (*((matA+i*sizeA[1]) + j))*scalar ;
                }
        }
}

/* 2X2 matrix inverse */

void MatInv2by2(const float *matA, float *matB){

	float det_inv = 1.0/ MatDet2by2((float *)matA);
  //unsigned int sizeA = {2,1};
	//MatScalarMult((float *)matA, sizeA, det_inv, (float *)matB);

  (*((matB) + 1)) = -(*((matA) + 1))*det_inv;
  (*((matB))) 		= (*((matA+2) + 1))*det_inv;
	(*((matB+2)))   = -(*((matA+2)))*det_inv;
	(*((matB+2)+1)) = (*((matA)))*det_inv;

}

float NormofMatrix(const float *matA, const unsigned int sizeA[]){
  float sum = 0.0;
  unsigned int i, j;
  for (i = 0; i < sizeA[0]; i++){
  	for (j = 0; j < sizeA[1]; j++){
    	sum += pow((*((matA+ i*sizeA[1]) + j)),2.0);
    }
  }   
  sum = pow(sum,0.5);
  return sum; 
}

/*Normalize Vector */
void NormalizeMatrix(const float *matA, const unsigned int sizeA[], float *matB){
  float norm =  NormofMatrix((float *) matA, sizeA);
  unsigned int i, j;
  for (i = 0; i < sizeA[0]; i++){
  	for (j = 0; j < sizeA[1]; j++){
    	(*((matB+ i*sizeA[1]) + j)) = (*((matA+ i*sizeA[1]) + j))/norm;
    }
  } 
}

/*Calculate Cross Skew Matrix */
void crossSkew(const float *matA, float *matB){
  (*(matB)) = 0;
  (*(matB+1)) = -(*(matA+2));
  (*(matB+2)) = (*(matA+1));

  (*(matB+3)) = (*(matA+2));
  (*(matB+4)) = 0;
  (*(matB+5)) = -(*(matA));

  (*(matB+6)) = -(*(matA+1));
  (*(matB+7)) = (*(matA));
  (*(matB+8)) = 0;

}

