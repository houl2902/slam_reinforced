
#include "MatrixFunctions.hpp"

constexpr double EPSILON = 1e-10;
constexpr double REPLACEMENT_VALUE = 1e-10; // Значение для замены проблемных чисел
constexpr double REPLACEMENT_VALUE_MAX = 1e+10; // Значение для замены проблемных чисел

MatrixOperations::MatrixOperations(){
    MatixFile.open("Matrix.txt");
    MatixFile.close();
}


void MatrixOperations::addMatrixToFile(Matrix& mat){
    MatixFile.open("Matrix.txt",std::ios::app);
    const int* size1 = mat.getSize();
    for (int i = 0; i < size1[0]; ++i) {
        for (int j = 0; j < size1[1]; ++j) {
            std::string matrix_logs = std::to_string(mat(i,j)) + " ";
        }
    }
};

void MatrixOperations::waitMatrixFromFile(Matrix& result){
    return;
};

inline bool MatrixOperations::isProblematic(double val) {
    return std::isnan(val) || std::isinf(val) || fabs(val) > std::numeric_limits<double>::max() / 2.0;
}

inline double MatrixOperations::safeValue(double val) {
    if (std::isnan(val)) return REPLACEMENT_VALUE;
    if (std::isinf(val)) return (val > 0) ? REPLACEMENT_VALUE_MAX : -REPLACEMENT_VALUE_MAX;
    return val;
}

void MatrixOperations::matrixMultiply(const Matrix& mat1, const Matrix& mat2, Matrix& result) {
    const int* size1 = mat1.getSize();
    const int* size2 = mat2.getSize();
    
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

void MatrixOperations::matrixTranspose(const Matrix& mat1, Matrix& result) {
    const int* size1 = mat1.getSize();
    
    for (int i = 0; i < size1[0]; ++i) {
        for (int j = 0; j < size1[1]; ++j) {
            double val = mat1(i,j);
            result(j,i) = safeValue(val);
        }
    }
}

void MatrixOperations::blockAdd(Matrix& H, int start_row, int start_col, Matrix& term) {
    const int* size1 = H.getSize();
    const int* size2 = term.getSize();
    for (int i = 0; i < size2[0]; ++i) {
        for (int j = 0; j < size2[1]; ++j) {
            if (start_row + i < size1[0] && start_col + j < size1[1]) {
                H(start_row + i,start_col + j) += term(i,j);
            }
        }
    }
}

void MatrixOperations::matrixAdd(const Matrix&  A, const Matrix&  B, Matrix& result) {
    const int* size1 = A.getSize();
    
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

void MatrixOperations::matrixSubtract(const Matrix&  A, const Matrix&  B, Matrix& result) {
    const int* size1 = A.getSize();
    
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


void MatrixOperations::matrixShow( Matrix& mat) {
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

void MatrixOperations::matrixShow( const Matrix& mat) const {
    const int* matSize = mat.getSize();
    for (int i = 0; i < matSize[0]; ++i) {
        for (int j = 0; j < matSize[1]; ++j) {
            double val = mat(i,j);
            // if (isProblematic(val)) {
            //     std::cout << "[NAN] ";
            // } else {
            //     std::cout << val << " ";
            // }
        }
        std::cout << std::endl;
    }
}

// namespace MatrixOps