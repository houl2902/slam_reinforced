cmake_minimum_required(VERSION 3.16)
project(slam LANGUAGES CXX)
# Опция для включения SDL
option(USE_SDL "Enable SDL visualization" OFF)
option(USE_PARALLEL "Enable parallel execution" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
message(STATUS "USE_SDL = ${USE_SDL}")
message(STATUS "USE_PARALLEL = ${USE_PARALLEL}")
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")


set(COMMON_SOURCES 
    main.cpp 
    src/Matrix.cpp 
    src/MatrixFunctions.cpp 
    src/EKFslam.cpp 
    src/Logger.cpp 
    src/GraphSLAM.cpp
)

if(USE_PARALLEL)
    list(APPEND SOURCES ${COMMON_SOURCES} src/ParrallelApp.cpp)
    add_definitions(-DUSE_PARALLEL)
    message(STATUS "Using parallel implementation: ParrallelApp.cpp")
elseif (USE_SDL)
    find_package(SDL2 REQUIRED) 
    include_directories(${SDL2_INCLUDE_DIR})
    add_definitions(-DUSE_SDL)
    list(APPEND SOURCES ${COMMON_SOURCES}  src/VisualApp.cpp)
    message(STATUS "SDL2 enabled - building with visualization")
else()
    list(APPEND SOURCES ${COMMON_SOURCES} src/App.cpp)
    message(STATUS "Using sequential implementation: App.cpp")
endif()

message(${SOURCES})
add_executable(slam ${SOURCES})
if(USE_SDL)
    target_link_libraries(slam ${SDL2_LIBRARIES})
endif()
