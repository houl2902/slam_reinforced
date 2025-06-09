#pragma once
#include <vector>
#include <algorithm>
#include <list>
#include <iostream>
#include <cmath>
#include <limits>
#include <fstream>
#include <stdexcept>
#include <string>
#include "Matrix.hpp"

class MatrixOperations {
   public:
   MatrixOperations();
   std::ofstream MatixFile;
   inline bool isProblematic(float val);
   inline float safeValue(float val);
   void blockAdd(Matrix&, int, int, Matrix&);
   void matrixMultiply(const Matrix& mat1, const Matrix& mat2, Matrix& result);
   void matrixTranspose( const Matrix& mat1, Matrix& result);
   void matrixAdd( const Matrix& A, const Matrix& B, Matrix& result);
   void matrixSubtract(const Matrix& A, const Matrix& B, Matrix&  result);
   void matrixShow(Matrix& mat);
   void matrixShow(const Matrix& mat) const;
   void addMatrixToFile(Matrix& mat);
   void waitMatrixFromFile(Matrix& result);
};