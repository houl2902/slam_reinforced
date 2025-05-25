#include <iostream>
#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <cmath>

struct Pose {
    int id;
    double x, y, theta;
    Pose(double _x, double _y, double _theta, int _id) : x(_x), y(_y), theta(_theta), id(_id) {}

    std::vector<double> toVector() const {
        return {x, y, theta};
    }
    void fromVector(const std::vector<double>& p_vec) {
        if (p_vec.size() == 3) {
            x = p_vec[0];
            y = p_vec[1];
            theta = p_vec[2];
        }
    }
};

struct Landmark {
    int id;
    double x, y;
    bool initialized;

    Landmark() : id(-1), x(0.0), y(0.0), initialized(false) {}
    Landmark(int _id, double _x, double _y) : id(_id), x(_x), y(_y), initialized(true) {}

    std::vector<double> toVector() const {
        return {x, y};
    }
    void fromVector(const std::vector<double>& lm_vec) {
        if (lm_vec.size() == 2) {
            x = lm_vec[0];
            y = lm_vec[1];
        }
    }
};

struct Observation {
    int pose_id;
    int landmark_id;
    double range;
    double bearing;

    Observation(int p_id, int l_id, double r, double b) : pose_id(p_id), landmark_id(l_id), range(r), bearing(b) {}
    std::vector<double> toVector() const {
        return {range, bearing};
    }
};

struct OdometryConstraint {
    int from_pose_id;
    int to_pose_id;
    double dx;
    double dy;
    double dtheta;

    OdometryConstraint(int from_id, int to_id, double _dx, double _dy, double _dtheta)
        : from_pose_id(from_id), to_pose_id(to_id), dx(_dx), dy(_dy), dtheta(_dtheta) {}
    std::vector<double> toVector() const {
        return {dx, dy, dtheta};
    }
};

class GraphSLAM
{
    public:
    // int num_poses_;    // Количество поз
    // int num_landmarks_;
    // Параметры для оптимизатора (шумы измерений)
    double sigma_odom_x = 0.1;
    double sigma_odom_y = 0.1;
    double sigma_odom_theta = 0.05;
    double sigma_obs_range = 0.1;
    double sigma_obs_bearing = 0.05;
    GraphSLAM();
    ~GraphSLAM() {
        for (Pose* p : history_poses_struct) delete p;
        history_poses_struct.clear();
        for (Landmark* l : landmarks) delete l;
        landmarks.clear();
        for (Observation* o : observations) delete o;
        observations.clear();
        for (OdometryConstraint* oc : odometry_constraints) delete oc;
        odometry_constraints.clear();
    };
    std::vector<double*> history_poses;
    std::vector<Pose*> history_poses_struct;
    std::vector<Landmark*> landmarks;
    std::vector<Observation*> observations;
    std::vector<Observation*> observations;
    std::vector<OdometryConstraint*> odometry_constraints;
    std::unordered_map<int, int> pose_id_to_index;  // Маппинг ID поз в индексы
    std::unordered_map<int, int> landmark_id_to_index;  // Маппинг ID landmarks в индексы
    double odometry_noise = 0.1;
    double observation_noise = 0.05;
    int next_pose_id = 0;
    int next_landmark_id = 0;
    void addPose(double* current_pos,double* measurements);
    void addPose(double x, double y, double theta);
    Pose* detectLoop(double* current_pos);
    void addLandmarkObservation(int pose_id, double range, double bearing);
    void optimizeGraph(int iterations = 10);
    void normalizeAngle(double& ang);
    void addLoopClosureConstraint(int current_pose_id, int matched_historical_pose_id);
    MatrixOperations matrixOps;
};