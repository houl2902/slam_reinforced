#include "src/VisualApp.hpp"
#include <iostream>

int main() {
    // Загрузка изображения
    std::cout << "Start app";
    
    VisualApp theApp;
    std::cout << "Start app2";
    return theApp.OnExecute();
   
}