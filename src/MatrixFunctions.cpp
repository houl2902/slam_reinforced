#include "MatrixFunctions.hpp"


void MatrixOperations::matrixMultiply( Matrix mat1, Matrix mat2, Matrix result){
    int* size1 = mat1.getSize();
    int* size2 = mat2.getSize();
    if (size1[1] != size2[0])  {
    std::cout << "Умножение невозможно: количество столбцов первой матрицы не равно количеству строк второй матрицы!" << std::endl;
    return;
    }
    for (int i = 0; i < size1[0]; ++i) {
        for (int j = 0; j < size2[1]; ++j) {
            for (int k = 0; k < size1[0]; ++k) {
                result(i,j) += mat1(i,k) * mat2(k,j);
            }
        }
    }
};

// void MatrixOperations::matrixTranspose( double A[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]){
//     for (int i = 0; i < MATRIX_SIZE; ++i) {
//         for (int j = 0; j < MATRIX_SIZE; ++j) {
//             result[j][i] = A[i][j];
//         }
//     }
// };

// void MatrixOperations::matrixAdd( double A[MATRIX_SIZE][MATRIX_SIZE],  double B[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]){
//     for (int i = 0; i < MATRIX_SIZE; ++i) {
//         for (int j = 0; j < MATRIX_SIZE; ++j) {
//             result[i][j] = A[i][j] + B[i][j];
//         }
//     }
// };

// void MatrixOperations::matrixSubtract( double A[MATRIX_SIZE][MATRIX_SIZE],  double B[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]){
//     for (int i = 0; i < MATRIX_SIZE; ++i) {
//         for (int j = 0; j < MATRIX_SIZE; ++j) {
//             result[i][j] = A[i][j] - B[i][j];
//         }
//     }
// };

// void MatrixOperations::matrixInverse( double A[MATRIX_SIZE][MATRIX_SIZE], double result[MATRIX_SIZE][MATRIX_SIZE]){
//     // Простая реализация для 3x3 матрицы
//     double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
//                  A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
//                  A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

//     if (det == 0) {
//         std::cerr << "Матрица вырождена, обратной не существует!" << std::endl;
//         return;
//     }

//     double invDet = 1.0 / det;

//     result[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
//     result[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
//     result[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;

//     result[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
//     result[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
//     result[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;

//     result[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
//     result[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
//     result[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;
// };


void MatrixOperations::matrixShow( Matrix mat){
    int* matSize = mat.getSize();
    for (int i = 0; i < matSize[0]; ++i) {
        for (int j = 0; j < matSize[1]; ++j) {
            std::cout<<mat(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
};

