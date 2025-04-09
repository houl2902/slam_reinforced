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

Matrix::~Matrix() {
    delete[] matPointer;
}

double& Matrix::operator() (int i, int j) {
    if (i < 0 || i >= sizeMat[0] || j < 0 || j >= sizeMat[1]) {
        std::cout<<"Индексы выходят за пределы матрицы!";
        throw;
    }
    return matPointer[i * sizeMat[1] + j];
};

// Функция изменения размера матрицы
void Matrix::resize(int newRows, int newCols) {
    if (newRows <= 0 || newCols <= 0) {
        throw std::invalid_argument("Matrix dimensions must be positive");
    }

    // Создаем новый массив
    double* newMat = new double[newRows * newCols]();
    
    // Копируем данные из старого массива в новый
    int minRows = std::min(sizeMat[0], newRows);
    int minCols = std::min(sizeMat[1], newCols);
    
    for (int i = 0; i < minRows; ++i) {
        for (int j = 0; j < minCols; ++j) {
            newMat[i * newCols + j] = matPointer[i * sizeMat[1] + j];
        }
    }
    
    // Освобождаем старую память
    delete[] matPointer;
    
    // Обновляем указатель и размеры
    matPointer = newMat;
    sizeMat[0] = newRows;
    sizeMat[1] = newCols;
}

// Добавим конструктор копирования и оператор присваивания для правильного управления памятью
Matrix::Matrix(const Matrix& other) {
    sizeMat[0] = other.sizeMat[0];
    sizeMat[1] = other.sizeMat[1];
    matPointer = new double[sizeMat[0] * sizeMat[1]];
    std::copy(other.matPointer, other.matPointer + sizeMat[0] * sizeMat[1], matPointer);
}

Matrix& Matrix::operator=(const Matrix& other) {
    if (this != &other) {
        delete[] matPointer;
        sizeMat[0] = other.sizeMat[0];
        sizeMat[1] = other.sizeMat[1];
        matPointer = new double[sizeMat[0] * sizeMat[1]];
        std::copy(other.matPointer, other.matPointer + sizeMat[0] * sizeMat[1], matPointer);
    }
    return *this;
}