cmake_minimum_required(VERSION 3.16)
project(slam LANGUAGES CXX)
# CMakeLists.txt
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
FetchContent_MakeAvailable(googletest)

add_executable(ekf_slam_tests test_ekf_slam.cpp /home/houl/slam_reinforced/src/Matrix.cpp /home/houl/slam_reinforced/src/MatrixFunctions.cpp /home/houl/slam_reinforced/src/EKFslam.cpp /home/houl/slam_reinforced/src/GraphSLAM.cpp)
target_link_libraries(ekf_slam_tests PRIVATE gtest_main)