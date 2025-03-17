#include <vector>
#include <algorithm>
#include <list>
#include <iostream>

static const int MATRIX_SIZE = 3;

class MatrixOperations {
   public:
     void matrixMultiply( double A[MATRIX_SIZE][MATRIX_SIZE],  double B[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]);
     void matrixTranspose( double A[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]);
     void matrixAdd( double A[MATRIX_SIZE][MATRIX_SIZE],  double B[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]);
     void matrixSubtract( double A[MATRIX_SIZE][MATRIX_SIZE],  double B[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]);
     void matrixInverse( double A[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]);
     void matrixShow(double Mat[MATRIX_SIZE][MATRIX_SIZE]);
};