
class EKFslam:
  private:
    const int STATE_SIZE;
    double state[STATE_SIZE];
    double covariance_matrix[STATE_SIZE][STATE_SIZE];


  