
#include "EKFslam.hpp"

EKFslam::EKFslam() : motion_noise(3, 3), covariance_matrix(3, 3), measurement_noise(2,2) {
    // Инициализация состояния (робот в начале координат)
    state[0] = 100; // x
    state[1] = 100; // y
    state[2] = 0.0001; // theta
    vel_noise_std = 0.01;    // Стандартное отклонение для линейной скорости
    ang_vel_noise_std = 0.005; 
    noisy_control[0] = 100;
    noisy_control[1] = 100;
    num_landmarks = 0;
    // Инициализация ковариационной матрицы
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            covariance_matrix(i,j) = (i == j) ? 0.01 : 0.0; // Диагональная матрица
        }
    }

    // Инициализация шумов
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            motion_noise(i,j) = (i == j) ? 0.1 : 0.0; // Диагональная матрица
        }
    }

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            measurement_noise(i,j) = (i == j) ? 0.01 : 0.0; // Диагональная матрица
        }
    }
}

void EKFslam::makeNoisyControl(double* control) {
    std::normal_distribution<double> vel_noise(0.0, vel_noise_std);
    std::normal_distribution<double> ang_vel_noise(0.0, ang_vel_noise_std);
    
    noisy_control[0] = control[0] + vel_noise(noise_generator);
    noisy_control[1] = control[1] + ang_vel_noise(noise_generator);
    motion_noise(0,0) = pow(vel_noise_std, 2);  // σ²_v
    motion_noise(1,1) = pow(vel_noise_std, 2);  // σ²_v
    motion_noise(2,2) = pow(ang_vel_noise_std, 2); // σ²_w
    control[0] = noisy_control[0];
    control[1] = noisy_control[1];
}

void EKFslam::makeNoisyMeasurement(double* measurement) {
    std::normal_distribution<double> vel_noise(0.0, 0.1);
    std::normal_distribution<double> ang_vel_noise(0.0, 0.004);
    
    double noisy_measurement_1 = measurement[0] + vel_noise(noise_generator);
    double noisy_measurement_2 = measurement[1] + ang_vel_noise(noise_generator);
    motion_noise(0,0) = pow(vel_noise_std, 2);  // σ²_v
    motion_noise(1,1) = pow(vel_noise_std, 2);  // σ²_v
    motion_noise(2,2) = pow(ang_vel_noise_std, 2); // σ²_w
    measurement[0] = noisy_measurement_1;
    measurement[1] = noisy_measurement_2;
}

void EKFslam::addLandmark(double x, double y) {
    if (num_landmarks >= MAX_LANDMARKS) {
        throw std::runtime_error("Maximum number of landmarks reached");
    }
    num_landmarks++;
    // Добавляем landmark в состояние
    int idx = 3 + 2*num_landmarks-2;
    state[idx] = x;
    state[idx+1] = y;
    std::cout << "RESIZE" << std::endl;
    std::cout << idx << std::endl;
    std::cout << 3 + 2*num_landmarks << std::endl;

    covariance_matrix.resize(3 + 2*num_landmarks,3 + 2*num_landmarks);
    
    // Инициализируем ковариацию для нового landmark
    covariance_matrix(idx,idx) = 1.0;
    covariance_matrix(idx+1,idx+1) = 1.0;
    covariance_matrix(idx, idx + 1) = 0.0;
    covariance_matrix(idx + 1, idx) = 0.0;
    for (int i = 0; i < idx; ++i) {
        covariance_matrix(i, idx) = 0.0;
        covariance_matrix(idx, i) = 0.0;
        covariance_matrix(i, idx + 1) = 0.0;
        covariance_matrix(idx + 1, i) = 0.0;
    }
    
    
}

void EKFslam::normalizeAngle(double& ang){
    double angle = fmod(ang + M_PI, 2*M_PI);
    ang = (angle < 0) ? angle + 2 * M_PI - M_PI : angle - M_PI;
}

void EKFslam::predict(double control[2]) {
    // Модель движения робота (только для робота, landmarks не двигаются)
    //state[2] = fmod(state[2] + M_PI, 2*M_PI) - M_PI;
    double dt = 1;
    double v = control[0];
    double w = control[1];
    
    makeNoisyControl(control);
    
    // Обновление состояния робота
    state[0] += v * std::cos(state[2]) * dt;
    state[1] += v * std::sin(state[2]) * dt;
    state[2] += w * dt;
    normalizeAngle(state[2]);

    std::cout << "STATE "<<std::endl;
    std::cout << v << std::endl;
    std::cout << w << std::endl;
    std::cout << v * std::cos(state[2])  << std::endl;
    std::cout << v * std::sin(state[2]) << std::endl;
    std::cout << state[2] << std::endl;
    // Матрица Якоби F (только для робота)
    Matrix F(3 + 2*num_landmarks, 3 + 2*num_landmarks);
    for (int i = 0; i < F.getSize()[0]; ++i) {
        F(i,i) = 1.0; // Единичная матрица по умолчанию
    }
    F(0,2) = -v * std::sin(state[2]) * dt;
    F(1,2) = v * std::cos(state[2]) * dt;
    
    // Обновление ковариации
    Matrix FT(F.getSize()[1], F.getSize()[0]);
    
    matrixOps.matrixTranspose(F, FT);
    
    Matrix temp(F.getSize()[0], FT.getSize()[1]);
    matrixOps.matrixMultiply(F, covariance_matrix, temp);
    matrixOps.matrixMultiply(temp, FT, covariance_matrix);
    for (int i = 0; i < 3; ++i) {
        covariance_matrix(i, i) += motion_noise(i, i);
    }

    // std::cout << "После predict:" << std::endl;
    // std::cout << "Состояние робота: [" 
    //           << state[0] << ", " << state[1] << ", " << state[2] << "]" << std::endl;
    // std::cout << "Первые 3x3 ковариации:\n";
    // for (int i = 0; i < 3; ++i) {
    //     for (int j = 0; j < 3; ++j) {
    //         std::cout << covariance_matrix(i,j) << " ";
    //     }
    //     std::cout << std::endl;
    // }
    
}

void EKFslam::update(double measurement[2],int landmark_id) {
    // Матрица Якоби для модели наблюдения (H)


    // 1. Проверка корректности landmark_id
    if (landmark_id < 0 || landmark_id >= num_landmarks) {
        throw std::invalid_argument("Invalid landmark ID");
    }
    std::cout << "MEASUREMENT BEFORE" << std::endl;
    std::cout << measurement[0] << std::endl;
    std::cout << measurement[1] << std::endl;

    makeNoisyMeasurement(measurement);

    std::cout << "MEASUREMENT AFTER" << std::endl;
    std::cout << measurement[0] << std::endl;
    std::cout << measurement[1] << std::endl;

    // 2. Получение текущего положения робота и landmark
    const double rx = state[0];
    const double ry = state[1];
    const double rt = state[2];
    const int lm_idx = 3 + 2 * landmark_id;
    const double lx = state[lm_idx];
    const double ly = state[lm_idx + 1];

    // 3. Вычисление разности координат
    const double dx = lx - rx;
    const double dy = ly - ry;
    const double q = dx * dx + dy * dy;

    // std::cout << "ABOBA1" << std::endl;
    // std::cout << lx << std::endl;
    // std::cout << ly << std::endl;
    // std::cout << rx << std::endl;
    // std::cout << ry << std::endl;
    // std::cout << dx << std::endl;
    // std::cout << dy << std::endl;
    // 4. Проверка на нулевое расстояние
    if (q < 1e-10) {
        std::cerr << "Warning: Robot is too close to landmark " << landmark_id << std::endl;
        return;
    }

    // 5. Создание матрицы Якоби H
    Matrix H(2, 3 + 2 * num_landmarks);
    
    // 5.1 Производные для робота
    H(0, 0) = -dx / sqrt(q);  // ∂r/∂x_robot
    H(0, 1) = -dy / sqrt(q);  // ∂r/∂y_robot
    H(0, 2) = 0.0;            // ∂r/∂θ_robot
    
    H(1, 0) = dy / q;         // ∂φ/∂x_robot
    H(1, 1) = -dx / q;        // ∂φ/∂y_robot
    H(1, 2) = -1.0;           // ∂φ/∂θ_robot

    // 5.2 Производные для landmark
    H(0, lm_idx) = dx / sqrt(q);     // ∂r/∂x_landmark
    H(0, lm_idx + 1) = dy / sqrt(q); // ∂r/∂y_landmark
    
    H(1, lm_idx) = -dy / q;          // ∂φ/∂x_landmark
    H(1, lm_idx + 1) = dx / q;       // ∂φ/∂y_landmark

    // 6. Предсказанное измерение
    double z_pred[2];
    z_pred[0] = sqrt(q);               // Расстояние
    z_pred[1] = atan2(dy, dx) - rt;    // Угол
    normalizeAngle(z_pred[1]);

    double der_ang = measurement[1] - z_pred[1];
    normalizeAngle(der_ang);
    // 7. Разность измерений
    double dz[2] = {
        measurement[0] - z_pred[0],
        der_ang
    };

    // 8. Вычисление матрицы усиления Калмана
    Matrix HT(3 + 2 * num_landmarks, 2);
    matrixOps.matrixTranspose(H, HT);
    Matrix S(2, 2);
    matrixOps.matrixShow(HT);
    Matrix temp(2, 3 + 2 * num_landmarks);
    matrixOps.matrixMultiply(H, covariance_matrix, temp);
    std::cout << "ABOBA" << std::endl;
    std::cout << z_pred[0] << std::endl;
    std::cout << z_pred[1] << std::endl;
    std::cout << measurement[0] << std::endl;
    std::cout << measurement[1] << std::endl;
    std::cout << dz[1] << std::endl;
    std::cout << dz[0] << std::endl;
    std::cout << state[3]  << std::endl;
    std::cout << state[4]  << std::endl;
    std::cout << "ABOBA2" << std::endl;
    matrixOps.matrixMultiply(temp, HT, S);
    Matrix S2(2, 2);
    matrixOps.matrixAdd(S, measurement_noise, S2);

    // 9. Проверка вырожденности S
    double det = S2(0, 0) * S2(1, 1) - S2(0, 1) * S2(1, 0);
    if (fabs(det) < 1e-10) {
        std::cerr << "Error: S matrix is singular! Adding regularization." << std::endl;
        S2(0, 0) += 1e-5; // Регуляризация
        S2(1, 1) += 1e-5;
        det = S2(0, 0) * S2(1, 1) - S2(0, 1) * S2(1, 0);
    }

    // 10. Обратная матрица S
    Matrix S_inv(2, 2);
    S_inv(0, 0) = S2(1, 1) / det;
    S_inv(0, 1) = -S2(0, 1) / det;
    S_inv(1, 0) = -S2(1, 0) / det;
    S_inv(1, 1) = S2(0, 0) / det;

    // 11. Коэффициент усиления Калмана
    Matrix K(3 + 2 * num_landmarks, 2);
    Matrix temp2(3 + 2 * num_landmarks, 2);
    matrixOps.matrixMultiply(covariance_matrix, HT, temp2);
    matrixOps.matrixMultiply(temp2, S_inv, K);

    // 12. Коррекция состояния
    for (int i = 0; i < 3 + 2 * num_landmarks; ++i) {
        state[i] += K(i, 0) * dz[0] + K(i, 1) * dz[1];
    }
    
    // 13. Нормализация угла робота
    normalizeAngle(state[2]);

    // 14. Обновление ковариации (Joseph form)
    Matrix I(3 + 2 * num_landmarks,3 + 2 * num_landmarks);
    Matrix KH(3 + 2 * num_landmarks, 3 + 2 * num_landmarks);
    matrixOps.matrixMultiply(K, H, KH);
    
    Matrix I_KH(3 + 2 * num_landmarks, 3 + 2 * num_landmarks);
    matrixOps.matrixSubtract(I, KH, I_KH);
    
    Matrix P_temp(3 + 2 * num_landmarks, 3 + 2 * num_landmarks);
    matrixOps.matrixMultiply(I_KH, covariance_matrix, P_temp);
    
    Matrix KRK(3 + 2 * num_landmarks, 3 + 2 * num_landmarks);
    Matrix temp3(3 + 2 * num_landmarks, 2);
    matrixOps.matrixMultiply(K, measurement_noise, temp3);
    Matrix KT(2, 3 + 2 * num_landmarks);
    matrixOps.matrixTranspose(K, KT);
    matrixOps.matrixMultiply(temp3, KT, KRK);
    
    matrixOps.matrixAdd(P_temp, KRK, covariance_matrix);
}