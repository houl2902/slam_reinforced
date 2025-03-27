#include <cmath>
#include <iostream>
#include "MatrixFunctions.hpp"

class EKFslam {
  
  private:
    EKFslam();
    MatrixOperations matrixOps;
    static const int STATE_SIZE = 3;
    static const int LANDMARK_SIZE = 2;
    double state[STATE_SIZE];
    Matrix covariance_matrix;
    Matrix motion_noise;
    Matrix measurement_noise;
    //void motionModel(double control[2]);
  public:
    void predict(double control[2]); 
    void update(double measurement[2]);
    void printState();
};
  