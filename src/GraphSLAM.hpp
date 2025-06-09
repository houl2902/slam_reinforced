#pragma once
#include <iostream>
#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <cmath>
#include <map>
#include <string>
#include <iomanip>
#include <numeric>     // For std::accumulate, std::inner_product
#include <algorithm>   // For std::transform (not used yet)
#include "MatrixFunctions.hpp"

inline void vector_segment_add(std::vector<double>& b, int start_idx, const std::vector<double>& term) {
    for (size_t i = 0; i < term.size(); ++i) {
        if (start_idx + i < b.size()) {
            b[start_idx + i] += term[i];
        }
    }
};

// Норма вектора
inline double vector_norm(const std::vector<double>& v) {
    double sum_sq = 0.0;
    for (double val : v) {
        sum_sq += val * val;
    }
    return std::sqrt(sum_sq);
};
// Умножение матрицы на вектор v_out = M * v_in
inline std::vector<double> matrix_vector_multiply(Matrix& M, const std::vector<double>& v_in) {
    if (M.getSize()[1] != v_in.size()) {
         std::cerr << "Ошибка: Несогласованные размеры матрицы и вектора для умножения!" << std::endl;
        return {};
    }
    size_t rows = M.getSize()[0];
    size_t cols = M.getSize()[1];
    std::vector<double> v_out(rows, 0.0);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            v_out[i] += M(i,j) * v_in[j];
        }
    }
    return v_out;
};

// Сложение векторов (result = a + b)
inline std::vector<double> vector_add(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) {
        std::cerr << "Ошибка: Размеры векторов для сложения не совпадают!" << std::endl;
        return {};
    }
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
};

// Вычитание векторов (result = a - b)
inline std::vector<double> vector_subtract(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) {
         std::cerr << "Ошибка: Размеры векторов для вычитания не совпадают!" << std::endl;
        return {};
    }
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] - b[i];
    }
    return result;
};

inline double vector_dot_product(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) {
        throw std::runtime_error("Vector dimensions mismatch for dot product. a_size=" + std::to_string(a.size()) + " b_size=" + std::to_string(b.size()));
    }
    // Using std::inner_product for efficiency and conciseness
    return std::inner_product(a.begin(), a.end(), b.begin(), 0.0);
};

inline std::vector<double> vector_scalar_multiply(const std::vector<double>& vec, double scalar) {
    std::vector<double> result = vec; // Make a copy
    for (double &val : result) {
        val *= scalar;
    }
    return result;
};

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
    std::vector<OdometryConstraint*> odometry_constraints;
    std::unordered_map<int, int> pose_id_to_index;  // Маппинг ID поз в индексы
    std::unordered_map<int, int> landmark_id_to_index;  // Маппинг ID landmarks в индексы
    double odometry_noise = 0.1;
    double observation_noise = 0.05;
    int next_pose_id = 0;
    int next_landmark_id = 0;
    Pose* addPose(double* current_pos,double* measurements);
    void addPose(double x, double y, double theta);
    Pose* detectLoop(double* current_pos);
    void addLandmarkObservation(int pose_id, double range, double bearing);
    void optimizeGraph(int iterations = 10);
    double normalizeAngle(double ang);
    void addLoopClosureConstraint(int current_pose_id, int matched_historical_pose_id);
    Pose* getPoseByIndex(size_t index);
    void addLandmark(Landmark* lm);
    void addOdometryConstraint(OdometryConstraint* oc);
    void addObservation(Observation* obs);
    MatrixOperations matrixOps;
};