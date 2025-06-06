#ifdef USE_SDL
#include "src/VisualApp.hpp"
#else
#include "src/App.hpp"
#endif
#include <iostream>

int main() {
    // Загрузка изображения
    std::cout << "Start app";

    
    //MatrixOperations m_func;
    EKFslam slam;
    GraphSLAM graph_slam;
    std::cout << "Start app2";
    #ifdef USE_SDL
    VisualApp theApp;
    #else
    App theApp;
    #endif
    return theApp.OnExecute(&slam,&graph_slam);
   
}