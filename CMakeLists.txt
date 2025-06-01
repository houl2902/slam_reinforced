cmake_minimum_required(VERSION 3.16)
project(slam LANGUAGES CXX)
# Опция для включения SDL
option(USE_SDL "Enable SDL visualization" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
message(STATUS "USE_SDL = ${USE_SDL}")
if (USE_SDL)
    find_package(SDL2 REQUIRED) 
    include_directories( ${SDL2_INCLUDE_DIR} )
    add_definitions(-DUSE_SDL)  # Определяем макрос USE_SDL
    add_executable(slam main.cpp src/VisualApp.cpp src/Matrix.cpp src/MatrixFunctions.cpp src/EKFslam.cpp src/Logger.cpp src/GraphSLAM.cpp)
    target_link_libraries( slam ${SDL2_LIBRARIES} )
else()
    add_executable(slam main.cpp src/App.cpp src/Matrix.cpp src/MatrixFunctions.cpp src/EKFslam.cpp src/Logger.cpp src/GraphSLAM.cpp)
endif()

