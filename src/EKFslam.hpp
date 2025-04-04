#include <cmath>
#include <iostream>
#include "MatrixFunctions.hpp"
#include <random> 

class EKFslam {
  
  private:
    MatrixOperations matrixOps;
    static const int STATE_SIZE = 3;
    static const int LANDMARK_SIZE = 2;
    Matrix covariance_matrix;
    Matrix motion_noise;
    Matrix measurement_noise;
    //void motionModel(double control[2]);
  public:
    EKFslam();
    std::default_random_engine noise_generator;
    double vel_noise_std;    // Стандартное отклонение для линейной скорости
    double ang_vel_noise_std; // Стандартное отклонение для угловой скорости
    double state[STATE_SIZE];
    void makeNoisyControl(double control[2]);
    void predict(double control[2]); 
    void update(double measurement[2]);
    void printState();
    double noisy_control[2];
};
  