#ifdef USE_SDL
#include "src/VisualApp.hpp"
#endif
#ifdef USE_PARALLEL
#include "src/ParrallelApp.hpp"
#endif
#if (!defined(USE_SDL) && !defined(USE_PARALLEL))
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
    #endif
    #ifdef USE_PARALLEL
    ParrallelApp theApp;
    #endif
    #if (!defined(USE_SDL) && !defined(USE_PARALLEL))
    App theApp;
    #endif

    return theApp.OnExecute(&slam,&graph_slam);
   
}