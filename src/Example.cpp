#include <vector>
#include <cmath>
#include <iostream>
#include <map>
#include <algorithm> // Для std::copy, std::find_if, std::fill
#include <limits>    // Для std::numeric_limits
#include <iomanip>   // Для std::setprecision

// --- Вспомогательные структуры (без Eigen) ---

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

// --- Вспомогательные функции для линейной алгебры (без Eigen) ---
// Инициализация матрицы нулями
void matrix_zeros(std::vector<std::vector<double>>& mat, int rows, int cols) {
    mat.assign(rows, std::vector<double>(cols, 0.0));
}

// Инициализация вектора нулями
void vector_zeros(std::vector<double>& vec, int size) {
    vec.assign(size, 0.0);
}

// Транспонирование матрицы
std::vector<std::vector<double>> matrix_transpose(const std::vector<std::vector<double>>& mat) {
    if (mat.empty()) return {};
    size_t rows = mat.size();
    size_t cols = mat[0].size();
    std::vector<std::vector<double>> T(cols, std::vector<double>(rows));
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            T[j][i] = mat[i][j];
        }
    }
    return T;
}

// Умножение матриц C = A * B
std::vector<std::vector<double>> matrix_multiply(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
    if (A.empty() || B.empty() || A[0].size() != B.size()) {
        // Несогласованные размеры, возвращаем пустую матрицу или выбрасываем исключение
        std::cerr << "Ошибка: Несогласованные размеры матриц для умножения!" << std::endl;
        return {};
    }
    size_t A_rows = A.size();
    size_t A_cols = A[0].size(); // == B_rows
    size_t B_cols = B[0].size();
    std::vector<std::vector<double>> C(A_rows, std::vector<double>(B_cols, 0.0));
    for (size_t i = 0; i < A_rows; ++i) {
        for (size_t j = 0; j < B_cols; ++j) {
            for (size_t k = 0; k < A_cols; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

// Умножение матрицы на вектор v_out = M * v_in
std::vector<double> matrix_vector_multiply(const std::vector<std::vector<double>>& M, const std::vector<double>& v_in) {
    if (M.empty() || M[0].size() != v_in.size()) {
         std::cerr << "Ошибка: Несогласованные размеры матрицы и вектора для умножения!" << std::endl;
        return {};
    }
    size_t rows = M.size();
    size_t cols = M[0].size();
    std::vector<double> v_out(rows, 0.0);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            v_out[i] += M[i][j] * v_in[j];
        }
    }
    return v_out;
}

// Сложение векторов (result = a + b)
std::vector<double> vector_add(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) {
        std::cerr << "Ошибка: Размеры векторов для сложения не совпадают!" << std::endl;
        return {};
    }
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}
// Вычитание векторов (result = a - b)
std::vector<double> vector_subtract(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) {
         std::cerr << "Ошибка: Размеры векторов для вычитания не совпадают!" << std::endl;
        return {};
    }
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] - b[i];
    }
    return result;
}

// Добавление к блоку матрицы (H_block += term)
void matrix_block_add(std::vector<std::vector<double>>& H, int start_row, int start_col, const std::vector<std::vector<double>>& term) {
    if (term.empty()) return;
    for (size_t i = 0; i < term.size(); ++i) {
        for (size_t j = 0; j < term[0].size(); ++j) {
            if (start_row + i < H.size() && start_col + j < H[0].size()) {
                H[start_row + i][start_col + j] += term[i][j];
            }
        }
    }
}
// Добавление к сегменту вектора (b_segment += term)
void vector_segment_add(std::vector<double>& b, int start_idx, const std::vector<double>& term) {
    for (size_t i = 0; i < term.size(); ++i) {
        if (start_idx + i < b.size()) {
            b[start_idx + i] += term[i];
        }
    }
}
// Норма вектора
double vector_norm(const std::vector<double>& v) {
    double sum_sq = 0.0;
    for (double val : v) {
        sum_sq += val * val;
    }
    return std::sqrt(sum_sq);
}


class GraphSLAM {
public:
    std::vector<Pose*> history_poses_struct;
    std::map<int, size_t> pose_id_to_index;
    int next_pose_id = 0;

    std::vector<Landmark*> landmarks;
    std::map<int, size_t> landmark_id_to_index;
    int next_landmark_id = 0;

    std::vector<Observation*> observations;
    std::vector<OdometryConstraint*> odometry_constraints;

    double new_pose_creation_distance_threshold = 1.0;
    double new_pose_creation_angle_threshold = 0.2;
    double landmark_association_max_distance = 1.5;
    double loop_closure_detection_distance = 2.0;

    double sigma_odom_x = 0.1;
    double sigma_odom_y = 0.1;
    double sigma_odom_theta = 0.05;
    double sigma_obs_range = 0.1;
    double sigma_obs_bearing = 0.05;

    GraphSLAM() = default;
    ~GraphSLAM() { /* ... (код деструктора без изменений) ... */ }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle <= -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void addPose(double* current_pos, double* measurements, int num_measurements);
    void addLandmarkObservation(int pose_id, double range, double bearing);
    int findAssociatedLandmark(double global_x, double global_y);
    Pose* detectLoop(double* current_pos_array);
    void addLoopClosureConstraint(int current_pose_id, int matched_historical_pose_id);
    void optimizeGraph(int iterations = 10);
};


void GraphSLAM::addPose(double* current_pos, double* measurements, int num_measurements) {
    Pose* new_pose = new Pose(current_pos[0], current_pos[1], normalizeAngle(current_pos[2]), next_pose_id++);
    bool add_new_pose_node = false;

    if (history_poses_struct.empty()) {
        add_new_pose_node = true;
    } else {
        const Pose* prev_pose = history_poses_struct.back();
        double dx_global = new_pose->x - prev_pose->x;
        double dy_global = new_pose->y - prev_pose->y;
        double dtheta_global = normalizeAngle(new_pose->theta - prev_pose->theta);
        double dist_moved = sqrt(dx_global * dx_global + dy_global * dy_global);

        if (dist_moved > new_pose_creation_distance_threshold || std::abs(dtheta_global) > new_pose_creation_angle_threshold) {
            add_new_pose_node = true;
            double cos_prev_theta = cos(-prev_pose->theta);
            double sin_prev_theta = sin(-prev_pose->theta);
            double dx_local = dx_global * cos_prev_theta - dy_global * sin_prev_theta;
            double dy_local = dx_global * sin_prev_theta + dy_global * cos_prev_theta;
            odometry_constraints.push_back(new OdometryConstraint(prev_pose->id, new_pose->id, dx_local, dy_local, dtheta_global));
            std::cout << "Добавлена одометрия: Из " << prev_pose->id << " В " << new_pose->id
                      << " (dx_local: " << dx_local << ", dy_local: " << dy_local << ", dtheta: " << dtheta_global << ")" << std::endl;
        }
    }

    if (add_new_pose_node) {
        history_poses_struct.push_back(new_pose);
        pose_id_to_index[new_pose->id] = history_poses_struct.size() - 1;
        std::cout << "Добавлена поза ID: " << new_pose->id << " в (" << new_pose->x << ", " << new_pose->y << ", " << new_pose->theta << ")" << std::endl;

        if (measurements != nullptr) {
            for (int i = 0; i < num_measurements; ++i) {
                 if ((i * 2 + 1) < num_measurements * 2) { // Предполагаем, что num_measurements - это количество пар
                     double range = measurements[i*2];
                     double bearing = measurements[i*2 + 1];
                     std::cout << "  Обработка измерения: Дальность=" << range << ", Азимут=" << bearing << std::endl;
                     addLandmarkObservation(new_pose->id, range, bearing);
                } else if (i == 0 && num_measurements == 1) { // Для совместимости с вашим исходным вызовом
                     double range = measurements[0];
                     double bearing = 0;
                     std::cout << "  Обработка одиночного измерения: Дальность=" << range << ", Азимут=" << bearing << std::endl;
                     addLandmarkObservation(new_pose->id, range, bearing);
                }
            }
        }
        Pose* loop_pose_candidate = detectLoop(current_pos);
        if (loop_pose_candidate != nullptr) {
            std::cout << "Обнаружен цикл! Текущая поза " << new_pose->id << " совпадает с исторической позой " << loop_pose_candidate->id << std::endl;
            addLoopClosureConstraint(new_pose->id, loop_pose_candidate->id);
        }
    } else {
        delete new_pose;
        next_pose_id--;
        std::cout << "Поза слишком близко к предыдущей, не добавлена." << std::endl;
    }
}
int GraphSLAM::findAssociatedLandmark(double global_x, double global_y) {
    int best_match_id = -1;
    double min_dist_sq = landmark_association_max_distance * landmark_association_max_distance;
    for (const auto& lm_ptr : landmarks) {
        if (!lm_ptr->initialized) continue;
        double dx = global_x - lm_ptr->x;
        double dy = global_y - lm_ptr->y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_match_id = lm_ptr->id;
        }
    }
    return best_match_id;
}

void GraphSLAM::addLandmarkObservation(int pose_id, double range, double bearing) {
    if (pose_id_to_index.find(pose_id) == pose_id_to_index.end()) {
        std::cerr << "ОШИБКА: ID позы " << pose_id << " не найден при добавлении наблюдения ориентира." << std::endl;
        return;
    }
    const Pose* current_pose = history_poses_struct[pose_id_to_index[pose_id]];
    double global_lm_x = current_pose->x + range * cos(current_pose->theta + bearing);
    double global_lm_y = current_pose->y + range * sin(current_pose->theta + bearing);
    int associated_lm_id = findAssociatedLandmark(global_lm_x, global_lm_y);
    Landmark* observed_landmark = nullptr;

    if (associated_lm_id != -1) {
        observed_landmark = landmarks[landmark_id_to_index[associated_lm_id]];
        std::cout << "  Наблюдение связано с существующим ориентиром ID: " << observed_landmark->id << std::endl;
    } else {
        observed_landmark = new Landmark(next_landmark_id++, global_lm_x, global_lm_y);
        landmarks.push_back(observed_landmark);
        landmark_id_to_index[observed_landmark->id] = landmarks.size() - 1;
        std::cout << "  Создан новый ориентир ID: " << observed_landmark->id << " в (" << global_lm_x << ", " << global_lm_y << ")" << std::endl;
    }
    observations.push_back(new Observation(pose_id, observed_landmark->id, range, bearing));
    std::cout << "  Добавлено наблюдение: Поза " << pose_id << " -> Ориентир " << observed_landmark->id << std::endl;
}

Pose* GraphSLAM::detectLoop(double* current_pos_array) {
    if (history_poses_struct.size() < 10) return nullptr;
    size_t check_up_to_index = history_poses_struct.size() > 5 ? history_poses_struct.size() - 5 : 0;
     if (check_up_to_index == 0 && history_poses_struct.size() > 1) check_up_to_index = history_poses_struct.size() -1; // Ensure we can check at least one if possible
    else if (check_up_to_index == 0) return nullptr;


    const Pose* current_eval_pose = history_poses_struct.back();
    for (size_t i = 0; i < check_up_to_index; ++i) {
        Pose* historical_pose = history_poses_struct[i];
        double dx = current_eval_pose->x - historical_pose->x;
        double dy = current_eval_pose->y - historical_pose->y;
        double dist_xy = sqrt(dx * dx + dy * dy);
        if (dist_xy < loop_closure_detection_distance) {
            return historical_pose;
        }
    }
    return nullptr;
}
void GraphSLAM::addLoopClosureConstraint(int current_pose_id, int matched_historical_pose_id) {
    if (pose_id_to_index.find(current_pose_id) == pose_id_to_index.end() ||
        pose_id_to_index.find(matched_historical_pose_id) == pose_id_to_index.end()) {
        std::cerr << "ОШИБКА: Неверный ID позы для ограничения замыкания цикла." << std::endl;
        return;
    }
    Pose* p_current = history_poses_struct[pose_id_to_index[current_pose_id]];
    Pose* p_hist = history_poses_struct[pose_id_to_index[matched_historical_pose_id]];
    double dx_global = p_current->x - p_hist->x;
    double dy_global = p_current->y - p_hist->y;
    double dtheta_global = normalizeAngle(p_current->theta - p_hist->theta);
    double cos_hist_theta = cos(-p_hist->theta);
    double sin_hist_theta = sin(-p_hist->theta);
    double dx_local = dx_global * cos_hist_theta - dy_global * sin_hist_theta;
    double dy_local = dx_global * sin_hist_theta + dy_global * cos_hist_theta;
    odometry_constraints.push_back(new OdometryConstraint(p_hist->id, p_current->id, dx_local, dy_local, dtheta_global));
    std::cout << "Добавлено ограничение ЗАМЫКАНИЯ ЦИКЛА: От исторической позы " << p_hist->id << " К текущей позе " << p_current->id
              << " (dx_local: " << dx_local << ", dy_local: " << dy_local << ", dtheta: " << dtheta_global << ")" << std::endl;
}

void GraphSLAM::optimizeGraph(int iterations) {
    std::cout << "\n--- Попытка оптимизации графа (без Eigen) ---" << std::endl;
    if (history_poses_struct.empty() || (odometry_constraints.empty() && observations.empty())) {
        std::cout << "Граф пуст или не имеет ограничений. Оптимизировать нечего." << std::endl;
        return;
    }
     if (history_poses_struct.size() < 1 && landmarks.empty()) { // Changed to < 1 for poses, as first pose is fixed
         std::cout << "Недостаточно данных для оптимизации." << std::endl;
        return;
    }


    int num_poses = history_poses_struct.size();
    int num_landmarks = landmarks.size();
    int total_vars = num_poses * 3 + num_landmarks * 2;

    std::cout << "Количество поз: " << num_poses << std::endl;
    std::cout << "Количество ориентиров: " << num_landmarks << std::endl;
    std::cout << "Количество ограничений одометрии/цикла: " << odometry_constraints.size() << std::endl;
    std::cout << "Количество наблюдений ориентиров: " << observations.size() << std::endl;
    std::cout << "Всего переменных для оптимизации: " << total_vars << std::endl;
    if (total_vars == 0) {
        std::cout << "Нет переменных для оптимизации." << std::endl;
        return;
    }


    // Информационные матрицы (обратные ковариациям)
    std::vector<std::vector<double>> omega_odom(3, std::vector<double>(3, 0.0));
    omega_odom[0][0] = 1.0 / (sigma_odom_x * sigma_odom_x);
    omega_odom[1][1] = 1.0 / (sigma_odom_y * sigma_odom_y);
    omega_odom[2][2] = 1.0 / (sigma_odom_theta * sigma_odom_theta);

    std::vector<std::vector<double>> omega_obs(2, std::vector<double>(2, 0.0));
    omega_obs[0][0] = 1.0 / (sigma_obs_range * sigma_obs_range);
    omega_obs[1][1] = 1.0 / (sigma_obs_bearing * sigma_obs_bearing);

    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<std::vector<double>> H;
        matrix_zeros(H, total_vars, total_vars);
        std::vector<double> b_vec; // Renamed from b to b_vec to avoid conflict
        vector_zeros(b_vec, total_vars);

        // Фиксируем первую позу (если она есть)
        if (num_poses > 0) {
             for(int i=0; i<3; ++i) H[i][i] += 1e6; // Большое значение для фиксации
        }


        // 1. Ограничения одометрии
        for (const auto& odom_constraint : odometry_constraints) {
            if (pose_id_to_index.find(odom_constraint->from_pose_id) == pose_id_to_index.end() ||
                pose_id_to_index.find(odom_constraint->to_pose_id) == pose_id_to_index.end()) {
                std::cerr << "Ошибка: ID позы в ограничении одометрии не найден в карте." << std::endl;
                continue;
            }
            int pose_i_idx = pose_id_to_index[odom_constraint->from_pose_id];
            int pose_j_idx = pose_id_to_index[odom_constraint->to_pose_id];

            Pose* p_i = history_poses_struct[pose_i_idx];
            Pose* p_j = history_poses_struct[pose_j_idx];

            std::vector<double> pose_i_vec = p_i->toVector();
            std::vector<double> pose_j_vec = p_j->toVector();
            std::vector<double> z_odom = odom_constraint->toVector();

            double xi = pose_i_vec[0], yi = pose_i_vec[1], thi = pose_i_vec[2];
            double xj = pose_j_vec[0], yj = pose_j_vec[1], thj = pose_j_vec[2];

            double pred_dx = (xj - xi) * cos(thi) + (yj - yi) * sin(thi);
            double pred_dy =-(xj - xi) * sin(thi) + (yj - yi) * cos(thi);
            double pred_dth = normalizeAngle(thj - thi);

            std::vector<double> error_odom(3);
            error_odom[0] = z_odom[0] - pred_dx;
            error_odom[1] = z_odom[1] - pred_dy;
            error_odom[2] = normalizeAngle(z_odom[2] - pred_dth);

            std::vector<std::vector<double>> A(3, std::vector<double>(3, 0.0));
            A[0][0] = -cos(thi); A[0][1] = -sin(thi); A[0][2] = -(xj - xi) * sin(thi) + (yj - yi) * cos(thi);
            A[1][0] =  sin(thi); A[1][1] = -cos(thi); A[1][2] = -(xj - xi) * cos(thi) - (yj - yi) * sin(thi);
            A[2][2] = -1.0;

            std::vector<std::vector<double>> B(3, std::vector<double>(3, 0.0));
            B[0][0] =  cos(thi); B[0][1] = sin(thi);
            B[1][0] = -sin(thi); B[1][1] = cos(thi);
            B[2][2] =  1.0;

            int idx_i = pose_i_idx * 3;
            int idx_j = pose_j_idx * 3;

            // H_ii += A^T * Omega * A
            matrix_block_add(H, idx_i, idx_i, matrix_multiply(matrix_transpose(A), matrix_multiply(omega_odom, A)));
            // H_jj += B^T * Omega * B
            matrix_block_add(H, idx_j, idx_j, matrix_multiply(matrix_transpose(B), matrix_multiply(omega_odom, B)));
            // H_ij += A^T * Omega * B
            matrix_block_add(H, idx_i, idx_j, matrix_multiply(matrix_transpose(A), matrix_multiply(omega_odom, B)));
            // H_ji += B^T * Omega * A
            matrix_block_add(H, idx_j, idx_i, matrix_multiply(matrix_transpose(B), matrix_multiply(omega_odom, A)));

            // b_i += A^T * Omega * e
            vector_segment_add(b_vec, idx_i, matrix_vector_multiply(matrix_transpose(A), matrix_vector_multiply(omega_odom, error_odom)));
            // b_j += B^T * Omega * e
            vector_segment_add(b_vec, idx_j, matrix_vector_multiply(matrix_transpose(B), matrix_vector_multiply(omega_odom, error_odom)));
        }

        // 2. Ограничения наблюдений ориентиров
        for (const auto& obs : observations) {
             if (pose_id_to_index.find(obs->pose_id) == pose_id_to_index.end() ||
                landmark_id_to_index.find(obs->landmark_id) == landmark_id_to_index.end()) {
                std::cerr << "Ошибка: ID позы или ориентира в наблюдении не найден в карте." << std::endl;
                continue;
            }
            int pose_idx = pose_id_to_index[obs->pose_id];
            int lm_idx = landmark_id_to_index[obs->landmark_id];

            Pose* p = history_poses_struct[pose_idx];
            Landmark* lm = landmarks[lm_idx];

            std::vector<double> pose_vec = p->toVector();
            std::vector<double> lm_vec = lm->toVector();
            std::vector<double> z_obs = obs->toVector();

            double px = pose_vec[0], py = pose_vec[1], pth = pose_vec[2];
            double lmx = lm_vec[0], lmy = lm_vec[1];

            double dx_lm = lmx - px; // Renamed from dx to dx_lm
            double dy_lm = lmy - py; // Renamed from dy to dy_lm
            double q = dx_lm*dx_lm + dy_lm*dy_lm;
            if (q < 1e-9) q = 1e-9; // Избегаем деления на ноль
            double sqrt_q = sqrt(q);

            std::vector<double> pred_obs(2);
            pred_obs[0] = sqrt_q;
            pred_obs[1] = normalizeAngle(atan2(dy_lm, dx_lm) - pth);

            std::vector<double> error_obs = vector_subtract(z_obs, pred_obs);
            error_obs[1] = normalizeAngle(error_obs[1]);

            std::vector<std::vector<double>> C(2, std::vector<double>(3, 0.0));
            C[0][0] = -dx_lm / sqrt_q; C[0][1] = -dy_lm / sqrt_q; C[0][2] = 0;
            C[1][0] =  dy_lm / q;     C[1][1] = -dx_lm / q;     C[1][2] = -1;

            std::vector<std::vector<double>> D(2, std::vector<double>(2, 0.0));
            D[0][0] = dx_lm / sqrt_q; D[0][1] = dy_lm / sqrt_q;
            D[1][0] = -dy_lm / q;     D[1][1] = dx_lm / q;

            int H_idx_p = pose_idx * 3;
            int H_idx_l = num_poses * 3 + lm_idx * 2;

            matrix_block_add(H, H_idx_p, H_idx_p, matrix_multiply(matrix_transpose(C), matrix_multiply(omega_obs, C)));
            matrix_block_add(H, H_idx_l, H_idx_l, matrix_multiply(matrix_transpose(D), matrix_multiply(omega_obs, D)));
            matrix_block_add(H, H_idx_p, H_idx_l, matrix_multiply(matrix_transpose(C), matrix_multiply(omega_obs, D)));
            matrix_block_add(H, H_idx_l, H_idx_p, matrix_multiply(matrix_transpose(D), matrix_multiply(omega_obs, C)));

            vector_segment_add(b_vec, H_idx_p, matrix_vector_multiply(matrix_transpose(C), matrix_vector_multiply(omega_obs, error_obs)));
            vector_segment_add(b_vec, H_idx_l, matrix_vector_multiply(matrix_transpose(D), matrix_vector_multiply(omega_obs, error_obs)));
        }

        // 3. Решение системы H * dx_vec_sol = b_vec (Упрощенный Гаусс-Зайдель)
        // ВНИМАНИЕ: Этот решатель очень базовый и может не сойтись или быть очень медленным.
        // Он требует, чтобы матрица H была диагонально доминирующей или симметричной положительно определенной.
        std::vector<double> dx_vec_sol(total_vars, 0.0);
        int solver_iterations = 100; // Количество итераций для решателя
        double solver_tolerance = 1e-5;

        for (int s_iter = 0; s_iter < solver_iterations; ++s_iter) {
            std::vector<double> prev_dx_vec_sol = dx_vec_sol;
            for (int i = 0; i < total_vars; ++i) {
                double sigma = 0.0;
                for (int j = 0; j < total_vars; ++j) {
                    if (i != j) {
                        sigma += H[i][j] * dx_vec_sol[j];
                    }
                }
                if (std::abs(H[i][i]) > 1e-9) { // Избегаем деления на ноль
                    dx_vec_sol[i] = (b_vec[i] - sigma) / H[i][i];
                } else {
                    dx_vec_sol[i] = 0; // Если диагональный элемент близок к нулю, это проблема
                }
            }
            if (vector_norm(vector_subtract(dx_vec_sol, prev_dx_vec_sol)) < solver_tolerance) {
                // std::cout << "  Решатель сошелся на итерации " << s_iter + 1 << std::endl;
                break;
            }
        }
        
        // 4. Обновление состояний
        for (int i = 0; i < num_poses; ++i) {
            std::vector<double> current_pose_val = history_poses_struct[i]->toVector();
            std::vector<double> update_segment(3);
            for(int k=0; k<3; ++k) update_segment[k] = dx_vec_sol[i * 3 + k];
            
            // Не обновляем первую позу, если она зафиксирована (ее dx будет ~0)
            // Однако, если мы хотим полностью зафиксировать, dx для нее должен быть принудительно 0.
            // В текущей реализации с большим H[0][0] ее dx будет очень мал.
            current_pose_val = vector_add(current_pose_val, update_segment);
            history_poses_struct[i]->fromVector(current_pose_val);
            history_poses_struct[i]->theta = normalizeAngle(history_poses_struct[i]->theta);
        }

        for (int i = 0; i < num_landmarks; ++i) {
            std::vector<double> current_lm_val = landmarks[i]->toVector();
            std::vector<double> update_segment(2);
             for(int k=0; k<2; ++k) update_segment[k] = dx_vec_sol[num_poses * 3 + i * 2 + k];

            current_lm_val = vector_add(current_lm_val, update_segment);
            landmarks[i]->fromVector(current_lm_val);
            if (!landmarks[i]->initialized) landmarks[i]->initialized = true;
        }

        double update_norm_val = vector_norm(dx_vec_sol);
        std::cout << "  Итерация " << iter + 1 << ", Норма обновления dx: " << update_norm_val << std::endl;
        if (update_norm_val < 1e-4) {
            std::cout << "  Сходимость достигнута." << std::endl;
            break;
        }
    }
    std::cout << "--- Оптимизация графа (без Eigen) завершена ---\n" << std::endl;
}


// --- Пример функции main ---
int main() {
    GraphSLAM slam_system;
    std::cout << std::fixed << std::setprecision(3); // Для красивого вывода

    // Моделируем движение робота и наблюдения
    double pose0_data[3] = {0.0, 0.0, 0.0};
    slam_system.addPose(pose0_data, nullptr, 0);

    double pose1_data[3] = {1.5, 0.1, M_PI/20.0};
    double measurements1[] = {3.0, 0.2};
    slam_system.addPose(pose1_data, measurements1, 1);

    double pose2_data[3] = {3.0, 0.3, M_PI/10.0};
    double measurements2[] = {2.8, 0.15, 4.0, -0.3};
    slam_system.addPose(pose2_data, measurements2, 2); // num_measurements здесь - это количество ПАР

    double pose3_data[3] = {4.5, 0.8, M_PI/8.0};
    double measurements3[] = {3.5, -0.25};
    slam_system.addPose(pose3_data, measurements3, 1);
    
    if (slam_system.history_poses_struct.size() > 1) { // Изменено, чтобы оптимизация запускалась раньше
        slam_system.optimizeGraph(10);
    }

    double poseN_data[3] = {0.2, 0.3, M_PI};
    double measurementsN[] = {3.1, M_PI/2.0}; 
    slam_system.addPose(poseN_data, measurementsN, 1);
    
    if (slam_system.odometry_constraints.size() > 3) { // Изменено условие
         slam_system.optimizeGraph(15);
    }

    std::cout << "\n--- Конечное состояние SLAM ---" << std::endl;
    std::cout << "Всего поз: " << slam_system.history_poses_struct.size() << std::endl;
    for(const auto& pose_ptr : slam_system.history_poses_struct) {
        std::cout << "Поза " << pose_ptr->id << ": (" << pose_ptr->x << ", " << pose_ptr->y << ", " << pose_ptr->theta << ")" << std::endl;
    }
    std::cout << "Всего ориентиров: " << slam_system.landmarks.size() << std::endl;
    for(const auto& lm_ptr : slam_system.landmarks) {
        if(lm_ptr->initialized) {
            std::cout << "Ориентир " << lm_ptr->id << ": (" << lm_ptr->x << ", " << lm_ptr->y << ")" << std::endl;
        } else {
            std::cout << "Ориентир " << lm_ptr->id << ": (Не инициализирован)" << std::endl;
        }
    }
    return 0;
}

