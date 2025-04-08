#include <cmath>
#include <limits>
#include <stdexcept>
#include "MatrixFunctions.hpp"

constexpr double EPSILON = 1e-10;
constexpr double REPLACEMENT_VALUE = 1e-10; // Значение для замены проблемных чисел
constexpr double REPLACEMENT_VALUE_MAX = 1e+10; // Значение для замены проблемных чисел



inline bool MatrixOperations::isProblematic(double val) {
    return std::isnan(val) || std::isinf(val) || fabs(val) > std::numeric_limits<double>::max() / 2.0;
}

inline double MatrixOperations::safeValue(double val) {
    if (std::isnan(val)) return REPLACEMENT_VALUE;
    if (std::isinf(val) || fabs(val) > std::numeric_limits<double>::max() / 2.0) return REPLACEMENT_VALUE_MAX;
    return val;
}

void MatrixOperations::matrixMultiply(Matrix mat1, Matrix mat2, Matrix result) {
    int* size1 = mat1.getSize();
    int* size2 = mat2.getSize();
    
    // Проверка размеров
    if (size1[1] != size2[0]) {
        throw std::invalid_argument("Matrix dimensions mismatch for multiplication");
    }

    // Инициализация результата нулями
    for (int i = 0; i < size1[0]; ++i) {
        for (int j = 0; j < size2[1]; ++j) {
            result(i,j) = 0.0;
        }
    }

    // Умножение с защитой от NaN/Inf
    for (int i = 0; i < size1[0]; ++i) {
        for (int j = 0; j < size2[1]; ++j) {
            double sum = 0.0;
            bool has_problem = false;
            
            for (int k = 0; k < size1[1]; ++k) {
                double a = safeValue(mat1(i,k));
                double b = safeValue(mat2(k,j));
                
                if (isProblematic(a)) {
                    std::cerr << "Warning: mat1(" << i << "," << k << ") = " << mat1(i,k) << std::endl;
                    has_problem = true;
                }
                if (isProblematic(b)) {
                    std::cerr << "Warning: mat2(" << k << "," << j << ") = " << mat2(k,j) << std::endl;
                    has_problem = true;
                }
                
                double product = a * b;
                if (isProblematic(product)) {
                    std::cerr << "Warning: product at (" << i << "," << j << ") = " << product << std::endl;
                    has_problem = true;
                    continue;
                }
                
                sum += product;
            }
            
            if (has_problem) {
                std::cerr << "Problem detected in row " << i << " col " << j 
                          << ", using safe value" << std::endl;
            }
            
            result(i,j) = safeValue(sum);
        }
    }
}

void MatrixOperations::matrixTranspose( Matrix mat1, Matrix result) {
    int* size1 = mat1.getSize();
    
    for (int i = 0; i < size1[0]; ++i) {
        for (int j = 0; j < size1[1]; ++j) {
            double val = mat1(i,j);
            result(j,i) = safeValue(val);
        }
    }
}

void MatrixOperations::matrixAdd( Matrix A,  Matrix B, Matrix result) {
    int* size1 = A.getSize();
    
    // Проверка размеров
    if (size1[0] != B.getSize()[0] || size1[1] != B.getSize()[1]) {
        throw std::invalid_argument("Matrix dimensions mismatch for addition");
    }

    for (int i = 0; i < size1[0]; ++i) {
        for (int j = 0; j < size1[1]; ++j) {
            double a = safeValue(A(i,j));
            double b = safeValue(B(i,j));
            double sum = a + b;
            
            result(i,j) = safeValue(sum);
        }
    }
}

void MatrixOperations::matrixSubtract( Matrix A,  Matrix B, Matrix result) {
    int* size1 = A.getSize();
    
    // Проверка размеров
    if (size1[0] != B.getSize()[0] || size1[1] != B.getSize()[1]) {
        throw std::invalid_argument("Matrix dimensions mismatch for subtraction");
    }

    for (int i = 0; i < size1[0]; ++i) {
        for (int j = 0; j < size1[1]; ++j) {
            double a = safeValue(A(i,j));
            double b = safeValue(B(i,j));
            double diff = a - b;
            
            result(i,j) = safeValue(diff);
        }
    }
}

// void matrixInverse(const Matrix& A, Matrix& result) {
//     int* size = A.getSize();
//     if (size[0] != size[1]) {
//         throw std::invalid_argument("Matrix must be square for inversion");
//     }

//     if (size[0] == 3) {
//         // Оптимизированная версия для 3x3
//         double a = safeValue(A(0,0)), b = safeValue(A(0,1)), c = safeValue(A(0,2));
//         double d = safeValue(A(1,0)), e = safeValue(A(1,1)), f = safeValue(A(1,2));
//         double g = safeValue(A(2,0)), h = safeValue(A(2,1)), i = safeValue(A(2,2));

//         double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
        
//         if (fabs(det) < EPSILON) {
//             throw std::runtime_error("Matrix is singular (determinant too small)");
//         }

//         double invDet = 1.0 / det;

//         result(0,0) = (e*i - f*h) * invDet;
//         result(0,1) = (c*h - b*i) * invDet;
//         result(0,2) = (b*f - c*e) * invDet;

//         result(1,0) = (f*g - d*i) * invDet;
//         result(1,1) = (a*i - c*g) * invDet;
//         result(1,2) = (c*d - a*f) * invDet;

//         result(2,0) = (d*h - e*g) * invDet;
//         result(2,1) = (b*g - a*h) * invDet;
//         result(2,2) = (a*e - b*d) * invDet;
//     } else {
//         // Общая реализация для других размеров
//         throw std::runtime_error("Matrix inversion not implemented for this size");
//     }
// }

void MatrixOperations::matrixShow( Matrix mat) {
    int* matSize = mat.getSize();
    for (int i = 0; i < matSize[0]; ++i) {
        for (int j = 0; j < matSize[1]; ++j) {
            double val = mat(i,j);
            if (isProblematic(val)) {
                std::cout << "[NAN] ";
            } else {
                std::cout << val << " ";
            }
        }
        std::cout << std::endl;
    }
}

// namespace MatrixOps