#include <vector>
#include <algorithm>
#include <list>
#include <iostream>
#include "Matrix.hpp"

class MatrixOperations {
   public:
   void matrixMultiply(Matrix mat1, Matrix mat2, Matrix result);
    //  void matrixTranspose3x3( double A[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]);
    //  void matrixAdd( double A[MATRIX_SIZE][MATRIX_SIZE],  double B[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]);
    //  void matrixSubtract( double A[MATRIX_SIZE][MATRIX_SIZE],  double B[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]);
    //  void matrixInverse( double A[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]);
   void matrixShow(Matrix mat);
};