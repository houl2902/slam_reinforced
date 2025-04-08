
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

void EKFslam::predict(double control[2]) {
    // Модель движения робота
    double dt = 1; // Временной шаг
    double v = control[0]; // Линейная скорость
    double w = control[1]; // Угловая скорость
    std::cout << "HERE00" << std::endl;
    std::cout << control[0] << std::endl;
    std::cout << control[1] << std::endl;
    makeNoisyControl(control);
    std::cout << "HERE11" << std::endl;
    std::cout << control[0] << std::endl;
    std::cout << control[1] << std::endl;


    // Обновление состояния
    std::cout << "__________________" << std::endl;
    std::cout << std::sin(state[2]) << std::endl;
    std::cout << "__________________" << std::endl;
    state[0] += v * std::cos(state[2]) * dt; // x
    state[1] += v * std::sin(state[2]) * dt; // y
    state[2] += w * dt; // theta
    std::cout << "HERE22" << std::endl;
    std::cout << state[0] << std::endl;
    std::cout << state[1] << std::endl;
    std::cout << state[2] << std::endl;
    
    //std::cout <<  << std::endl;
    std::cout << "++++++++++++++++++++" << std::endl;
    Matrix F(3,3);
    // Матрица Якоби для модели движения (F)
    F(0,0) = 1;
    F(0,2) = -v * std::sin(state[2]) * dt;
    F(1,1) = 1;
    F(1,2) = v * std::cos(state[2]) * dt;
    F(2,2) = 1;
    std::cout << "HERE444" << std::endl;
    matrixOps.matrixShow(F);
    
    // Обновление ковариации
    Matrix FT(3,3);
    matrixOps.matrixTranspose(F, FT);
    std::cout << "HERE441" << std::endl;
    matrixOps.matrixShow(FT);
    Matrix temp(3,3);
    matrixOps.matrixMultiply(F, covariance_matrix, temp);
    std::cout << "HERE43" << std::endl;
    matrixOps.matrixShow(temp);
    matrixOps.matrixShow(FT);
    matrixOps.matrixMultiply(temp, FT, covariance_matrix);
    matrixOps.matrixShow(covariance_matrix);
    std::cout << "HERE42" << std::endl;
    matrixOps.matrixShow(covariance_matrix);
    matrixOps.matrixAdd(covariance_matrix, motion_noise, covariance_matrix);
    std::cout << "HERE44" << std::endl;
    matrixOps.matrixShow(covariance_matrix);
    matrixOps.matrixShow(F);
    // delete[] temp.getMatrix(); 
    // delete[] F.getMatrix(); 
    // delete[] FT.getMatrix(); 
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
    
    H(0,0) = -std::cos(theta);
    H(0,1) = -std::sin(theta);
    H(1,0) = std::sin(theta) / r_squared;
    H(1,1) = -std::cos(theta) / r_squared;
    H(1,2) = -1;
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
    std::cout << "HERE" << std::endl;
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
    for (int i = 0; i < STATE_SIZE; ++i) {
        state[i] += K(i,0) * dz[0] + K(i,1) * dz[1];
    }
    

    // Коррекция ковариации
    Matrix I(3,3);
    Matrix KH(3,3);
    Matrix temp3(3,3);
    matrixOps.matrixMultiply(K, H, KH);
    matrixOps.matrixSubtract(I, KH, temp3);
    matrixOps.matrixMultiply(temp3, covariance_matrix, covariance_matrix);
    state[2] = std::fmod(state[2] + M_PI, 2*M_PI) - M_PI;
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