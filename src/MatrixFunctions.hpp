#include <vector>
#include <algorithm>
#include <list>
#include <iostream>
#include "Matrix.hpp"

class MatrixOperations {
   public:
   inline bool isProblematic(double val);
   inline double safeValue(double val);
   void matrixMultiply(Matrix mat1, Matrix mat2, Matrix result);
   void matrixTranspose( Matrix mat1, Matrix result);
   void matrixAdd( Matrix A, Matrix B, Matrix result);
   void matrixSubtract(Matrix A, Matrix B, Matrix result);
   void matrixShow(Matrix mat);
};