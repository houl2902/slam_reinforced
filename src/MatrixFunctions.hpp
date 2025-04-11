#include <vector>
#include <algorithm>
#include <list>
#include <iostream>
#include "Matrix.hpp"

class MatrixOperations {
   public:
   inline bool isProblematic(double val);
   inline double safeValue(double val);
   void matrixMultiply(const Matrix& mat1, const Matrix& mat2, Matrix& result);
   void matrixTranspose( const Matrix& mat1, Matrix& result);
   void matrixAdd( const Matrix& A, const Matrix& B, Matrix& result);
   void matrixSubtract(const Matrix& A, const Matrix& B, Matrix&  result);
   void matrixShow(Matrix& mat);
   void matrixShow(const Matrix& mat) const;
};