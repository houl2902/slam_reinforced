#include "src/VisualApp.hpp"
#include "src/MatrixFunctions.hpp"
#include <iostream>

int main() {
    // Загрузка изображения
    std::cout << "Start app";
    MatrixOperations m_func;
    VisualApp theApp;
    double matrix_res[MATRIX_SIZE][MATRIX_SIZE] = {0};
    double matrix_A[MATRIX_SIZE][MATRIX_SIZE] = {{1.0,3.0,4.0},{1.0,3.0,4.0},{1.0,3.0,4.0}};
    double matrix_B[MATRIX_SIZE][MATRIX_SIZE] = {{1,2,3},{1,2,3},{1,2,3}};
    m_func.matrixMultiply(matrix_A,matrix_B,matrix_res);
    m_func.matrixInverse(matrix_A,matrix_res);
    m_func.matrixShow(matrix_res);
    std::cout << "Start app2";
    return theApp.OnExecute();
   
}