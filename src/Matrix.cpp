#include "Matrix.hpp"
#include <iostream>
Matrix::Matrix(int size[2]){
    sizeMat[0] = size[0];
    sizeMat[1] = size[1];
    matPointer = new double[sizeMat[0] * sizeMat[1]];
};

Matrix::Matrix(int x, int y){
    sizeMat[0] = x;
    sizeMat[1] = y;
    matPointer = new double[x * y];
};
int* Matrix::getSize(){
    return sizeMat;
};

double* Matrix::getMatrix(){
    return matPointer;
};

// Matrix::~Matrix() {
//     delete[] matPointer;  // Освобождаем память
// }

double& Matrix::operator() (int i, int j) {
    if (i < 0 || i >= sizeMat[0] || j < 0 || j >= sizeMat[1]) {
        std::cout<<"Индексы выходят за пределы матрицы!";
        throw;
    }
    return matPointer[i * sizeMat[1] + j];
};