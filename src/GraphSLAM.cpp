#include "GraphSLAM.hpp"


void GraphSLAM::addPose(double* current_pos){

    double* copy_pos = new double[5];  // Предполагаем 5 элементов
    std::copy(current_pos, current_pos + 5, copy_pos);
    if (history_poses.empty()) {
        history_poses.push_back(copy_pos);
        return;
    };
    const auto& prev_pose = history_poses.back();
    double dx = current_pos[0] - prev_pose[0];
    double dy = current_pos[1] - prev_pose[1];
    double length = sqrt(dx*dx + dy*dy);
    if (length>25) history_poses.push_back(copy_pos);
    return;
};

double* GraphSLAM::detectLoop(double* current_pos){
    if (history_poses.size() < 10) return nullptr;
    for (size_t i = 0; i < history_poses.size()-1; ++i){
        double* prev_pose = history_poses[i];
        double dx = current_pos[0] - prev_pose[0];
        double dy = current_pos[1] - prev_pose[1];
        double dist_xy = sqrt(dx*dx + dy*dy);
        if ((dist_xy < 5) &&
        (std::abs(current_pos[3] - prev_pose[3]) < 5 && std::abs(current_pos[4] - prev_pose[4]) < 5)){
            std::cout << "LOOP HERE" << std::endl;
            std::cout << history_poses.size() << std::endl;
            return prev_pose;
            
        };
    };
    return nullptr;
};

// GraphSLAM::GraphSLAM(int pose_dim = 3, int landmark_dim = 2) 
//     : omega(Matrix(pose_dim, pose_dim)), 
//       xi(Matrix(pose_dim, 1)),
//       num_poses(0),
//       num_landmarks(0) {}

// // Инициализация системы для N поз и M ориентиров
// void GraphSLAM::initialize(int n_poses, int n_landmarks) {
//     int dim = n_poses * 3 + n_landmarks * 2;
//     omega = Matrix(dim, dim);
//     xi = Matrix(dim, 1);
//     num_poses = n_poses;
//     num_landmarks = n_landmarks;
// }

// // Добавление одометрического ограничения между позами i и j
// void GraphSLAM::addOdometryConstraint(int i, int j, const Matrix& z, const Matrix& Q) {
//     Matrix Q_inv(2, 2);
//     try {
//         // Здесь должна быть реализация инвертирования Q
//         // Для простоты предположим, что Q_inv уже вычислена
//         Q_inv(0,0) = 1.0/Q(0,0); Q_inv(1,1) = 1.0/Q(1,1);
//     } catch (...) {
//         throw std::runtime_error("Failed to invert odometry covariance matrix");
//     }
    
//     // Обновляем информационную матрицу
//     for (int k = 0; k < 3; ++k) {
//         for (int l = 0; l < 3; ++l) {
//             omega(i*3+k, i*3+l) += Q_inv(k,l);
//             omega(j*3+k, j*3+l) += Q_inv(k,l);
//             omega(i*3+k, j*3+l) -= Q_inv(k,l);
//             omega(j*3+k, i*3+l) -= Q_inv(k,l);
//         }
//     }
    
//     // Обновляем информационный вектор
//     for (int k = 0; k < 3; ++k) {
//         xi(i*3+k, 0) += Q_inv(k,0)*z(0,0) + Q_inv(k,1)*z(1,0);
//         xi(j*3+k, 0) -= Q_inv(k,0)*z(0,0) + Q_inv(k,1)*z(1,0);
//     }
// }

// // Добавление ограничения измерения между позой и ориентиром
// void GraphSLAM::addLandmarkConstraint(int pose_id, int landmark_id, const Matrix& z, const Matrix& Q) {
//     Matrix Q_inv(2, 2);
//     try {
//         // Инвертирование матрицы ковариации
//         Q_inv(0,0) = 1.0/Q(0,0); Q_inv(1,1) = 1.0/Q(1,1);
//     } catch (...) {
//         throw std::runtime_error("Failed to invert measurement covariance matrix");
//     }
    
//     int pose_offset = pose_id * 3;
//     int landmark_offset = num_poses * 3 + landmark_id * 2;
    
//     // Обновление информационной матрицы
//     for (int k = 0; k < 3; ++k) {
//         for (int l = 0; l < 2; ++l) {
//             omega(pose_offset + k, landmark_offset + l) += Q_inv(k,l);
//             omega(landmark_offset + l, pose_offset + k) += Q_inv(k,l);
//         }
//     }
    
//     // Обновление информационного вектора
//     for (int k = 0; k < 2; ++k) {
//         xi(landmark_offset + k, 0) += Q_inv(k,0)*z(0,0) + Q_inv(k,1)*z(1,0);
//     }
// }

// // Решение системы и получение оценок поз и ориентиров
// Matrix GraphSLAM::solve() {
//     try {
//         // Создаем копию информационной матрицы для решения
//         Matrix omega_copy = omega;
//         Matrix xi_copy = xi;
//         Matrix L(omega_.getSize()[0], omega_.getSize()[1]);
//         Matrix D(omega_.getSize()[0], 1);
        
//         // Выполняем LDLT разложение
//         // if (!ldltDecomposition(omega_, L, D)) {
//         //     throw std::runtime_error("LDLT decomposition failed");
//         // }
        
//         // Решаем систему: L*D*L^T * x = xi
//         // 1. Решаем L*y = xi (прямая подстановка)
//         Matrix y = forwardSubstitution(L, xi_);
        
//         // 2. Решаем D*z = y
//         for (int i = 0; i < y.getSize()[0]; ++i) {
//             y(i,0) /= D(i,0);
//         }

//         // 3. Решаем L^T*x = z (обратная подстановка)
//         Matrix x = backwardSubstitution(L.transpose(), y);
        
//         return x;
//     } 
//     catch (const std::exception& e) {
//         std::cerr << "SLAM solve error: " << e.what() << std::endl;
//         // Попытка использовать SVD как fallback
//         try {
//             Matrix U, S, V;
//             if (!svdDecomposition(omega_, U, S, V)) {
//                 throw std::runtime_error("SVD fallback also failed");
//             }
            
//             // Псевдообратная матрица: V * S^-1 * U^T
//             Matrix Sinv(S.getSize()[1], S.getSize()[0]);
//             for (int i = 0; i < std::min(S.getSize()[0], S.getSize()[1]); ++i) {
//                 Sinv(i,i) = (fabs(S(i,i)) > 1e-6) ? 1.0/S(i,i) : 0.0;
//             }
            
//             Matrix omega_inv = V * Sinv * U.transpose();
//             return omega_inv * xi_;
//         }
//         catch (...) {
//             throw std::runtime_error("All matrix solving methods failed");
//         }
    
//         // Здесь должна быть реализация решения системы линейных уравнений
//         // Например, с использованием декомпозиции Холецкого или других методов
        
//         // В реальной реализации здесь будет вызов решателя:
//         // Matrix mu = omega_copy.solve(xi_copy);
        
//         // Для примера возвращаем копию xi
//         return xi_copy;
//     } 
//     catch (...) {
//         throw std::runtime_error("Failed to solve GraphSLAM system");
//     }
// }

// // Оптимизация с использованием итерационного метода
// void GraphSLAM::optimize(int iterations = 10) {
//     Matrix mu = solve(); // Начальное решение
    
//     for (int iter = 0; iter < iterations; ++iter) {
//         // Линеаризация вокруг текущего решения
//         linearize(mu);
        
//         // Решение обновленной системы
//         mu = solve();
//     }
// }

// // Линеаризация системы вокруг текущего решения
// void GraphSLAM::linearize(const Matrix& mu) {
//     // Временные матрицы для линеаризации
//     Matrix H_odom(3, 3);    // Якобиан для одометрии
//     Matrix H_landmark(2, 5); // Якобиан для измерений ориентиров
    
//     // Линеаризация одометрических ограничений
//     for (auto& constraint : odometry_constraints_) {
//         int i = constraint.i;
//         int j = constraint.j;
        
//         // Получаем текущие оценки поз
//         double xi = mu(i*pose_dim, 0);
//         double yi = mu(i*pose_dim+1, 0);
//         double thetai = mu(i*pose_dim+2, 0);
//         double xj = mu(j*pose_dim, 0);
//         double yj = mu(j*pose_dim+1, 0);
        
//         // Вычисляем ожидаемое измерение
//         double dx = xj - xi;
//         double dy = yj - yi;
//         double predicted_dist = sqrt(dx*dx + dy*dy);
//         double predicted_angle = atan2(dy, dx) - thetai;
        
//         // Якобиан по i-й позе
//         H_odom(0,0) = -dx/predicted_dist;  // ∂dist/∂xi
//         H_odom(0,1) = -dy/predicted_dist;   // ∂dist/∂yi
//         H_odom(0,2) = 0;                    // ∂dist/∂θi
//         H_odom(1,0) = dy/(predicted_dist*predicted_dist);  // ∂angle/∂xi
//         H_odom(1,1) = -dx/(predicted_dist*predicted_dist); // ∂angle/∂yi
//         H_odom(1,2) = -1;                   // ∂angle/∂θi
        
//         // Обновляем информационную матрицу
//         Matrix Ht = H_odom.transpose();
//         Matrix temp = Ht * constraint.Q_inv * H_odom;
        
//         for (int k = 0; k < pose_dim; ++k) {
//             for (int l = 0; l < pose_dim; ++l) {
//                 omega_(i*pose_dim+k, i*pose_dim+l) += temp(k,l);
//                 omega_(j*pose_dim+k, j*pose_dim+l) += temp(k,l);
//                 omega_(i*pose_dim+k, j*pose_dim+l) -= temp(k,l);
//                 omega_(j*pose_dim+k, i*pose_dim+l) -= temp(k,l);
//             }
//         }
//     }
    
//     // Линеаризация измерений ориентиров
//     for (auto& observation : landmark_observations_) {
//         int pose_idx = observation.pose_id;
//         int lm_idx = observation.landmark_id;
        
//         // Текущие оценки позы и ориентира
//         double x = mu(pose_idx*pose_dim, 0);
//         double y = mu(pose_idx*pose_dim+1, 0);
//         double theta = mu(pose_idx*pose_dim+2, 0);
//         double lx = mu(num_poses_*pose_dim + lm_idx*landmark_dim, 0);
//         double ly = mu(num_poses_*pose_dim + lm_idx*landmark_dim+1, 0);
        
//         // Ожидаемое измерение
//         double dx = lx - x;
//         double dy = ly - y;
//         double range = sqrt(dx*dx + dy*dy);
//         double bearing = atan2(dy, dx) - theta;
        
//         // Якобиан по позе и ориентиру
//         H_landmark(0,0) = -dx/range;       // ∂range/∂x
//         H_landmark(0,1) = -dy/range;       // ∂range/∂y
//         H_landmark(0,2) = 0;                // ∂range/∂θ
//         H_landmark(0,3) = dx/range;         // ∂range/∂lx
//         H_landmark(0,4) = dy/range;         // ∂range/∂ly
        
//         H_landmark(1,0) = dy/(range*range); // ∂bearing/∂x
//         H_landmark(1,1) = -dx/(range*range);// ∂bearing/∂y
//         H_landmark(1,2) = -1;               // ∂bearing/∂θ
//         H_landmark(1,3) = -dy/(range*range);// ∂bearing/∂lx
//         H_landmark(1,4) = dx/(range*range); // ∂bearing/∂ly
        
//         // Обновление информационной матрицы
//         Matrix Ht = H_landmark.transpose();
//         Matrix temp = Ht * observation.Q_inv * H_landmark;
        
//         // Индексы в большой матрице
//         int pose_offset = pose_idx * pose_dim;
//         int lm_offset = num_poses_ * pose_dim + lm_idx * landmark_dim;
        
//         for (int k = 0; k < pose_dim; ++k) {
//             for (int l = 0; l < pose_dim; ++l) {
//                 omega_(pose_offset+k, pose_offset+l) += temp(k,l);
//             }
//             for (int l = 0; l < landmark_dim; ++l) {
//                 omega_(pose_offset+k, lm_offset+l) += temp(k, pose_dim+l);
//                 omega_(lm_offset+l, pose_offset+k) += temp(pose_dim+l, k);
//             }
//         }
        
//         for (int k = 0; k < landmark_dim; ++k) {
//             for (int l = 0; l < landmark_dim; ++l) {
//                 omega_(lm_offset+k, lm_offset+l) += temp(pose_dim+k, pose_dim+l);
//             }
//         }
//     }
// }

// // Визуализация текущего состояния
// void GraphSLAM::visualize() const {
//     std::cout << "Information Matrix (Omega):" << std::endl;
//     ops.matrixShow(omega);
    
//     std::cout << "Information Vector (xi):" << std::endl;
//     ops.matrixShow(xi);
// }
