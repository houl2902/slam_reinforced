#include "src/VisualApp.hpp"
#include <iostream>

int main() {
    // Загрузка изображения
    std::cout << "Start app";

    VisualApp theApp;
    MatrixOperations m_func;
    EKFslam slam;
    std::cout << "Start app2";
    return theApp.OnExecute(&slam);
   
}