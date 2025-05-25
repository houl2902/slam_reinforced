#include <cmath>
#include <iostream>

#include <random> 

class EKFslam {
  
  public:
    MatrixOperations matrixOps;
    static const int STATE_SIZE = 3;
    static const int MAX_LANDMARKS = 10;
    double curr_measurement[2];
    Matrix covariance_matrix;
    Matrix motion_noise;
    Matrix measurement_noise;
    //void motionModel(double control[2]);
    int num_landmarks;
    EKFslam();
    void addLandmark(double x, double y);
    std::default_random_engine noise_generator;
    double vel_noise_std;    // Стандартное отклонение для линейной скорости
    double ang_vel_noise_std; // Стандартное отклонение для угловой скорости
    double state[STATE_SIZE+2*MAX_LANDMARKS];
    void makeNoisyMeasurement(double* measurement);
    // double was_states[STATE_SIZE];
    void normalizeAngle(double& ang);
    void makeNoisyControl(double* control);
    void predict(double control[2]); 
    void update(double measurement[2],int landmark_id);
    void printState();
    double noisy_control[2];
};
  