#include <cmath>
#include <iostream>
#include "MatrixFunctions.hpp"

class EKFslam {

  private:
    EKFslam();
    static const int STATE_SIZE = 3;
    static const int LANDMARK_SIZE = 2;
    double state[STATE_SIZE];
    double covariance_matrix[STATE_SIZE][STATE_SIZE];
    double motion_noise[STATE_SIZE][STATE_SIZE];
    double measurement_noise[LANDMARK_SIZE][LANDMARK_SIZE];
    //void motionModel(double control[2]);
  public:
    // void predict(double control[2]); 
    // void update(double measurement[2]);
    void printState();
};
  