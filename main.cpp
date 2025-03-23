#include "src/VisualApp.hpp"
#include "src/MatrixFunctions.hpp"
#include <iostream>

int main() {
    // Загрузка изображения
    std::cout << "Start app";
    MatrixOperations m_func;
    VisualApp theApp;
    
    // double matrix_res[MATRIX_SIZE][MATRIX_SIZE] = {0};
    // double matrix_A[MATRIX_SIZE][MATRIX_SIZE] = {{1.0,3.0,4.0},{1.0,3.0,4.0},{1.0,3.0,4.0}};
    // double matrix_B[MATRIX_SIZE][MATRIX_SIZE] = {{1,2,3},{1,2,3},{1,2,3}};
    int m_size[2] = {3,3};
    Matrix matrix_A(m_size);
    Matrix matrix_B(m_size);
    Matrix res(m_size);

    // Заполняем матрицу данными
    for (int i = 0; i < matrix_A.getSize()[0]; ++i) {
        for (int j = 0; j < matrix_A.getSize()[1]; ++j) {
            matrix_A(i, j) = i * matrix_A.getSize()[1] + j+0.1;  // Используем operator()
        }
    }
    // Заполняем матрицу данными
    for (int i = 0; i < matrix_B.getSize()[0]; ++i) {
        for (int j = 0; j < matrix_B.getSize()[1]; ++j) {
            matrix_B(i, j) = i * matrix_B.getSize()[1] + j+0.2;  // Используем operator()
        }
    }
    m_func.matrixMultiply(matrix_A,matrix_B,res);
    //m_func.matrixInverse(matrix_A,matrix_res);
    m_func.matrixShow(res);
    std::cout << "Start app2";
    return theApp.OnExecute();
   
}