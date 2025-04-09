
#include "EKFslam.hpp"

EKFslam::EKFslam() : motion_noise(3, 3), covariance_matrix(3, 3), measurement_noise(2,2) {
    // Инициализация состояния (робот в начале координат)
    state[0] = 100; // x
    state[1] = 100; // y
    state[2] = 0.0001; // theta
    vel_noise_std = 0.1;    // Стандартное отклонение для линейной скорости
    ang_vel_noise_std = 0.05; 
    noisy_control[2];
    was_states[0] = 100;
    was_states[1] = 100;
    was_states[2] = 0.0001;
    num_landmarks = 0;
    // Инициализация ковариационной матрицы
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            covariance_matrix(i,j) = (i == j) ? 0.1 : 0.0; // Диагональная матрица
        }
    }

    // Инициализация шумов
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            motion_noise(i,j) = (i == j) ? 0.01 : 0.0; // Диагональная матрица
        }
    }

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            measurement_noise(i,j) = (i == j) ? 0.1 : 0.0; // Диагональная матрица
        }
    }
}

void EKFslam::makeNoisyControl(double control[2]) {
    std::normal_distribution<double> vel_noise(0.0, vel_noise_std);
    std::normal_distribution<double> ang_vel_noise(0.0, ang_vel_noise_std);
    
    noisy_control[0] = control[0] + vel_noise(noise_generator);
    noisy_control[1] = control[1] + ang_vel_noise(noise_generator);
    motion_noise(0,0) = pow(vel_noise_std, 2);  // σ²_v
    motion_noise(1,1) = pow(vel_noise_std, 2);  // σ²_v
    motion_noise(2,2) = pow(ang_vel_noise_std, 2); // σ²_w
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

void EKFslam::predict(double control[2]) {
    // Модель движения робота (только для робота, landmarks не двигаются)
    double dt = 1;
    double v = control[0];
    double w = control[1];
    
    makeNoisyControl(control);
    
    // Обновление состояния робота
    state[0] += v * std::cos(state[2]) * dt;
    state[1] += v * std::sin(state[2]) * dt;
    state[2] += w * dt;
    
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
    // 7. Отладочный вывод (можно закомментировать)

    std::cout << "После predict:" << std::endl;
    std::cout << "Состояние робота: [" 
              << state[0] << ", " << state[1] << ", " << state[2] << "]" << std::endl;
    std::cout << "Первые 3x3 ковариации:\n";
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            std::cout << covariance_matrix(i,j) << " ";
        }
        std::cout << std::endl;
    }
    
}

void EKFslam::update(double measurement[2]) {
    // Матрица Якоби для модели наблюдения (H)


    const double theta = state[2];
    const double x = state[0], y = state[1];
    double r_squared = x*x + y*y;
    
    if (fabs(r_squared) < 1e-10) {
    r_squared = (x*x + y*y < 0) ? -1e-10 : 1e-10;
    }
    Matrix H(2,3);
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "r_squared: " << r_squared << std::endl;
    std::cout << "theta: " << theta << std::endl;
    
    H(0, 0) = 1.0; H(0, 1) = 0.0; H(0, 2) = 0.0;  // ∂x_measure/∂x
    H(1, 0) = 0.0; H(1, 1) = 1.0; H(1, 2) = 0.0;  // ∂y_measure/∂y

    //std::cout << r_squared << std::endl;
    double z_pred[2];
    // Предсказанное измерение
    z_pred[0] = std::sqrt(r_squared);         // Предсказанное расстояние
    z_pred[1] = std::atan2(y, x) - theta;     // Предсказанный угол
    
    matrixOps.matrixShow(H);
    // Матрица усиления Калмана (K)
    Matrix HT(3,2);
    matrixOps.matrixTranspose(H, HT);

    Matrix S(2,2);
    Matrix temp(2,3);
    matrixOps.matrixMultiply(H, covariance_matrix, temp);
    matrixOps.matrixMultiply(temp, HT, S);
    matrixOps.matrixAdd(S, measurement_noise, S);

    Matrix S_inv(2,2);
    // Вычисление обратной матрицы S (для простоты используем явную формулу для 2x2)
    double det = S(0,0) * S(1,1) - S(0,1) * S(1,0);
    
    //std::cout << measurements[1] << std::endl;
    if (std::abs(det) < 1e-10) {
        std::cerr << "Матрица S вырождена!" << "(det = " << det << ")"  << std::endl;
        return;
    }
    S_inv(0,0) = S(1,1) / det;
    S_inv(0,1) = -S(0,1) / det;
    S_inv(1,0) = -S(1,0) / det;
    S_inv(1,1) = S(0,0) / det;
    
    Matrix K(3,2);
    Matrix temp2(3,2);
    // matrixOps.matrixShow(covariance_matrix);
    // matrixOps.matrixShow(HT);
    matrixOps.matrixMultiply(covariance_matrix, HT, temp2);
    // std::cout << "HERE3" << std::endl;
    // matrixOps.matrixShow(temp2);
    // matrixOps.matrixShow(HT);
    // matrixOps.matrixShow(covariance_matrix);
    // matrixOps.matrixShow(S_inv);
    matrixOps.matrixMultiply(temp2, S_inv, K);
    // std::cout << "HERE2" << std::endl;

    // Коррекция состояния
    double dz[2] = {measurement[0] - z_pred[0], measurement[1] - z_pred[1]};
    // 8. Коррекция состояния (только x и y)
    state[0] += K(0,0)*dz[0] + K(0,1)*dz[1];
    state[1] += K(1,0)*dz[0] + K(1,1)*dz[1];
    // Угол не обновляется (так как нет информации об угле)

    // 9. Обновление ковариации (упрощенная форма)
    Matrix I(3, 3);
    for (int i = 0; i < 3; ++i) I(i,i) = 1.0;
    
    Matrix KH(3, 3);
    matrixOps.matrixMultiply(K, H, KH);
    
    Matrix I_KH(3, 3);
    matrixOps.matrixSubtract(I, KH, I_KH);
    
    matrixOps.matrixMultiply(I_KH, covariance_matrix, covariance_matrix);
    // matrixOps.matrixMultiply(K, measurement_noise, temp4);
    // matrixOps.matrixTranspose(K, K.transpose(), KRK);
    
    // matrixOps.matrixAdd(P_temp2, KRK, covariance_matrix);
    //state[2] = std::fmod(state[2] + M_PI, 2*M_PI) - M_PI;
    // delete [] KH.getMatrix();
    // delete [] I.getMatrix();
    // delete [] temp.getMatrix();
    // delete [] temp2.getMatrix();
    // delete [] temp3.getMatrix();
    // delete [] K.getMatrix();
    // delete [] S_inv.getMatrix();
    // delete [] S.getMatrix();
    // delete [] H.getMatrix();
}