#include "GraphSLAM.hpp"



Pose* GraphSLAM::addPose(double* current_pos,double* measurements){
    
    Pose* new_pose = new Pose(current_pos[0], current_pos[1], current_pos[2], next_pose_id++);

    // double* copy_pos = new double[5];  // Предполагаем 5 элементов
    // std::copy(current_pos, current_pos + 5, copy_pos);
    if (history_poses_struct.empty()) {

        history_poses_struct.push_back(new_pose);
        pose_id_to_index[new_pose->id] = history_poses_struct.size() - 1;
        // history_poses.push_back(copy_pos);
        return new_pose;
    };
    const auto& prev_pose = history_poses_struct.back();
    double dx = new_pose->x - prev_pose->x;
    double dy = new_pose->y - prev_pose->y;
    double length = sqrt(dx*dx + dy*dy);
    
    if (length>25) {
      history_poses_struct.push_back(new_pose);
      pose_id_to_index[new_pose->id] = history_poses_struct.size() - 1;
      
      double range = measurements[0];
      int num_measurements = 1;
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
        
    } else {
        delete new_pose;
        next_pose_id--;
        //std::cout << "Поза слишком близко к предыдущей, не добавлена." << std::endl;
    }
    return new_pose;
};


double GraphSLAM::normalizeAngle(double ang){
    double angle = fmod(ang + M_PI, 2*M_PI);
    angle = (angle < 0) ? angle + 2 * M_PI - M_PI : angle - M_PI;
    return angle;
};

void GraphSLAM::addLandmark(Landmark* lm) {
    if (!lm) return;
    landmark_id_to_index[lm->id] = static_cast<int>(landmarks.size());
    landmarks.push_back(lm);
};

void GraphSLAM::addOdometryConstraint(OdometryConstraint* oc) {
    if (!oc) return;
    odometry_constraints.push_back(oc);
};

void GraphSLAM::addObservation(Observation* obs) { 
    if (!obs) return;
    observations.push_back(obs); 
};

void GraphSLAM::addLandmarkObservation(int pose_id, double range, double bearing) {
    // Предполагаем, что это новый landmark
    Landmark* new_landmark = new Landmark();
    new_landmark->id = next_landmark_id++;
    
    // Вычисляем абсолютные координаты landmark
    const Pose* pose = history_poses_struct[pose_id_to_index[pose_id]];
    new_landmark->x = pose->x + range * cos(pose->theta + bearing);
    new_landmark->y = pose->y + range * sin(pose->theta + bearing);
    
    
    landmarks.push_back(new_landmark);
    landmark_id_to_index[new_landmark->id] = landmarks.size() - 1;
    
    // Добавляем наблюдение
    observations.push_back(new Observation(pose_id, new_landmark->id, range, bearing));
    std::cout << "HERE2 second"<< std::endl;
    std::cout << observations[0]->range << std::endl;
};

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
};

Pose* GraphSLAM::detectLoop(double* current_pos){
    if (history_poses_struct.size() < 1) return nullptr;
    for (size_t i = 0; i < history_poses_struct.size()-1; ++i){
        Pose* prev_pose = history_poses_struct[i];
        double dx = current_pos[0] - prev_pose->x;
        double dy = current_pos[1] - prev_pose->y;
        double dist_xy = sqrt(dx*dx + dy*dy);
        std::cout << "LOOP MAYBE HERE" << std::endl;
        std::cout << dist_xy << std::endl;
        if (dist_xy < 8){
            std::cout << "LOOP HERE" << std::endl;
            std::cout << history_poses_struct.size() << std::endl;
            return prev_pose;

        };
    };
    return nullptr;
};

Pose* GraphSLAM::getPoseByIndex(size_t index) { // Use size_t for index
    if (index >= history_poses_struct.size()) return nullptr;
    return history_poses_struct[index];
}

void GraphSLAM::optimizeGraph(int iterations) {
    std::cout << "\n--- Попытка оптимизации графа ---" << std::endl;
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
    Matrix omega_odom(3, 3);
    omega_odom(0,0) = 1.0 / (sigma_odom_x * sigma_odom_x);
    omega_odom(1,1) = 1.0 / (sigma_odom_y * sigma_odom_y);
    omega_odom(2,2) = 1.0 / (sigma_odom_theta * sigma_odom_theta);

    Matrix omega_obs(2, 2);
    omega_obs(0,0) = 1.0 / (sigma_obs_range * sigma_obs_range);
    omega_obs(1,1) = 1.0 / (sigma_obs_bearing * sigma_obs_bearing);

    for (int iter = 0; iter < iterations; ++iter) {
        Matrix H(total_vars,total_vars);
        std::vector<double> b_vec(total_vars,0.0); // Renamed from b to b_vec to avoid conflict

        // Фиксируем первую позу (если она есть)
        if (num_poses > 0) {
             for(int i=0; i<3; ++i) H(i,i) += 1e6; // Большое значение для фиксации
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

            Matrix A(3, 3);
            A(0,0) = -cos(thi); A(0,1) = -sin(thi); A(0,2) = -(xj - xi) * sin(thi) + (yj - yi) * cos(thi);
            A(1,0) =  sin(thi); A(1,1) = -cos(thi); A(1,2) = -(xj - xi) * cos(thi) - (yj - yi) * sin(thi);
            A(2,2) = -1.0;

            Matrix B(3, 3);
            B(0,0) =  cos(thi); B(0,1) = sin(thi);
            B(1,0) = -sin(thi); B(1,1) = cos(thi);
            B(2,2) =  1.0;

            int idx_i = pose_i_idx * 3;
            int idx_j = pose_j_idx * 3;

            Matrix AT(3, 3);
            Matrix BT(3, 3);
            // Обновление ковариации
    
            matrixOps.matrixTranspose(A, AT);
            matrixOps.matrixTranspose(B, BT);

            // H_ii += A^T * Omega * A
            Matrix temp(3, 3);
            Matrix temp2(3, 3);
            matrixOps.matrixMultiply(omega_odom,A,temp);
            matrixOps.matrixMultiply(AT,temp,temp2);
            matrixOps.blockAdd(H,idx_i, idx_i,temp2);
            // H_jj += B^T * Omega * B
            Matrix temp3(3, 3);
            Matrix temp4(3, 3);
            matrixOps.matrixMultiply(omega_odom,B,temp3);
            matrixOps.matrixMultiply(BT,temp3,temp4);
            matrixOps.blockAdd(H,idx_j, idx_j,temp4);
            // H_ij += A^T * Omega * B
            Matrix temp5(3, 3);
            Matrix temp6(3, 3);
            matrixOps.matrixMultiply(omega_odom,B,temp5);
            matrixOps.matrixMultiply(AT,temp5,temp6);
            matrixOps.blockAdd(H,idx_i, idx_j,temp6);
            // H_ji += B^T * Omega * A
            Matrix temp7(3, 3);
            Matrix temp8(3, 3);
            matrixOps.matrixMultiply(omega_odom,A,temp7);
            matrixOps.matrixMultiply(BT,temp7,temp8);
            matrixOps.blockAdd(H,idx_j, idx_i,temp8);

            // b_i += A^T * Omega * e
            vector_segment_add(b_vec, idx_i, matrix_vector_multiply(AT, matrix_vector_multiply(omega_odom, error_odom)));
            // b_j += B^T * Omega * e
            vector_segment_add(b_vec, idx_j, matrix_vector_multiply(BT, matrix_vector_multiply(omega_odom, error_odom)));
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

            Matrix C(2, 3);
            C(0,0) = -dx_lm / sqrt_q; C(0,1) = -dy_lm / sqrt_q; C(0,2) = 0;
            C(1,0) =  dy_lm / q;     C(1,1) = -dx_lm / q;     C(1,2) = -1;

            Matrix D(2, 2);
            D(0,0) = dx_lm / sqrt_q; D(0,1) = dy_lm / sqrt_q;
            D(1,0) = -dy_lm / q;     D(1,1) = dx_lm / q;

            int H_idx_p = pose_idx * 3;
            int H_idx_l = num_poses * 3 + lm_idx * 2;

            Matrix DT(2, 2);
            Matrix CT(3, 2);
            matrixOps.matrixTranspose(C, CT);
            matrixOps.matrixTranspose(D, DT);

            Matrix obs_temp(2, 3);
            Matrix obs_temp2(3, 3);
            matrixOps.matrixMultiply(omega_obs,C,obs_temp);
            matrixOps.matrixMultiply(CT,obs_temp,obs_temp2);
            matrixOps.blockAdd(H, H_idx_p, H_idx_p, obs_temp2);

            //matrix_block_add(H, H_idx_p, H_idx_p, matrix_multiply(matrix_transpose(C), matrix_multiply(omega_obs, C)));
            Matrix obs_temp3(2, 3);
            Matrix obs_temp4(3, 3);
            matrixOps.matrixMultiply(omega_obs,D,obs_temp3);
            matrixOps.matrixMultiply(DT,obs_temp3,obs_temp4);
            matrixOps.blockAdd(H, H_idx_l, H_idx_l, obs_temp4);

            //matrix_block_add(H, H_idx_p, H_idx_l, matrix_multiply(matrix_transpose(C), matrix_multiply(omega_obs, D)));
            Matrix obs_temp5(2, 3);
            Matrix obs_temp6(3, 3);
            matrixOps.matrixMultiply(omega_obs,D,obs_temp5);
            matrixOps.matrixMultiply(CT,obs_temp5,obs_temp6);
            matrixOps.blockAdd(H, H_idx_p, H_idx_l, obs_temp6);
            //matrix_block_add(H, H_idx_l, H_idx_p, matrix_multiply(matrix_transpose(D), matrix_multiply(omega_obs, C)));
            Matrix obs_temp7(2, 3);
            Matrix obs_temp8(3, 3);
            matrixOps.matrixMultiply(omega_obs,C,obs_temp7);
            matrixOps.matrixMultiply(DT,obs_temp7,obs_temp8);
            matrixOps.blockAdd(H, H_idx_l, H_idx_p, obs_temp8);

            vector_segment_add(b_vec, H_idx_p, matrix_vector_multiply(CT, matrix_vector_multiply(omega_obs, error_obs)));
            vector_segment_add(b_vec, H_idx_l, matrix_vector_multiply(DT, matrix_vector_multiply(omega_obs, error_obs)));
        }

        // 3. Решение системы H * dx_vec_sol = b_vec (Упрощенный Гаусс-Зайдель)
        // 3. Решение системы H * dx_vec_sol = b_vec (Метод сопряженных градиентов)
        std::vector<double> dx_vec_sol(total_vars, 0.0); // Начальное приближение x_0 = 0
        std::vector<double> r = b_vec; // r_0 = b - A*x_0. Так как x_0 = 0, r_0 = b_vec
        std::vector<double> p = r;     // p_0 = r_0
        double rsold = vector_dot_product(r, r);
        
        int solver_iterations = std::min(total_vars, 100); // Макс. итераций для CG (может быть total_vars)
        double solver_tolerance_sq = 1e-10; // Используем квадрат нормы невязки для сравнения (1e-5)^2

        bool solver_converged = false;
        if (std::sqrt(rsold) < 1e-9) { // Если b_vec (и следовательно r_0) очень мал, решение уже найдено (dx_vec_sol=0)
            std::cout << "  Решатель (CG): Начальная невязка мала, решение dx=0." << std::endl;
            solver_converged = true;
        } else {
            for (int s_iter = 0; s_iter < solver_iterations; ++s_iter) {
                std::vector<double> Ap = matrix_vector_multiply(H, p);
                double p_dot_Ap = vector_dot_product(p, Ap);
                
                double alpha;
                if (std::abs(p_dot_Ap) < 1e-12) { // Избегаем деления на ноль или очень маленькое число
                                                 // Это может произойти, если p ортогонален Ap, или p близок к нулю
                    std::cout << "  Решатель (CG): p_dot_Ap близко к нулю на итерации " << s_iter + 1 << ". Остановка." << std::endl;
                    // Если p_dot_Ap == 0, то либо p=0 (что означает, что r=0, и мы должны были сойтись),
                    // либо H не положительно определена на направлении p.
                    // В этом случае текущее dx_vec_sol - лучшее, что мы можем сделать с этим p.
                    break; 
                }
                alpha = rsold / p_dot_Ap;

                dx_vec_sol = vector_add(dx_vec_sol, vector_scalar_multiply(p, alpha));
                r = vector_subtract(r, vector_scalar_multiply(Ap, alpha));
                
                double rsnew = vector_dot_product(r, r);
                if (std::sqrt(rsnew) < std::sqrt(solver_tolerance_sq)) { // Проверка сходимости по норме невязки
                    std::cout << "  Решатель (CG) сошелся на итерации " << s_iter + 1 << " с невязкой " << std::sqrt(rsnew) << std::endl;
                    solver_converged = true;
                    break;
                }
                
                p = vector_add(r, vector_scalar_multiply(p, rsnew / rsold));
                rsold = rsnew;

                if (s_iter == solver_iterations - 1 && !solver_converged) {
                    std::cout << "  Решатель (CG) достиг максимального количества итераций (" << solver_iterations << ") с невязкой " << std::sqrt(rsnew) << std::endl;
                }
            }
        }
       
        
        // 4. Обновление состояний
        for (int i = 0; i < num_poses; ++i) {
            std::vector<double> current_pose_val = history_poses_struct[i]->toVector();
            std::vector<double> update_segment(3);
            for(int k=0; k<3; ++k) update_segment[k] = dx_vec_sol[i * 3 + k];
            std::cout << "  Обновление состояний1  " << i << " "<<  update_segment[0] << " " << update_segment[1] << " " << update_segment[2] << " "  << std::endl;
            std::cout << "  Обновление состояний2  " << i << " " <<  current_pose_val[0] << " " << current_pose_val[1] << " " << current_pose_val[2]<< " " << std::endl;
            // Не обновляем первую позу, если она зафиксирована (ее dx будет ~0)
            // Однако, если мы хотим полностью зафиксировать, dx для нее должен быть принудительно 0.
            // В текущей реализации с большим H[0][0] ее dx будет очень мал.
            current_pose_val = vector_add(current_pose_val, update_segment);
            history_poses_struct[i]->fromVector(current_pose_val);
            history_poses_struct[i]->theta = normalizeAngle(history_poses_struct[i]->theta);
            std::cout << "  Обновление состояний3  " << i << " " <<  history_poses_struct[i]->x<< " " << history_poses_struct[i]->y << " "<< history_poses_struct[i]->theta<< " " << std::endl;
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
    std::cout << "--- Оптимизация графа  завершена ---\n" << std::endl;
}