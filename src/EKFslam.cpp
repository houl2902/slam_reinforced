
#include "EKFslam.hpp"

EKFslam::EKFslam() : motion_noise(3, 3), covariance_matrix(3, 3), measurement_noise(2,2) {
    // Инициализация состояния (робот в начале координат)
    state[0] = 0.0; // x
    state[1] = 0.0; // y
    state[2] = 0.0; // theta

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

void EKFslam::predict(double control[2]) {
    // Модель движения робота
    double dt = 1.0; // Временной шаг
    double v = control[0]; // Линейная скорость
    double w = control[1]; // Угловая скорость

    // Обновление состояния
    state[0] += v * std::cos(state[2]) * dt; // x
    state[1] += v * std::sin(state[2]) * dt; // y
    state[2] += w * dt; // theta

    Matrix F(3,3);
    // Матрица Якоби для модели движения (F)
    F(0,0) = 1;
    F(0,2) = -v * std::sin(state[2]) * dt;
    F(1,1) = 1;
    F(1,2) = v * std::cos(state[2]) * dt;
    F(2,2) = 1;

    // Обновление ковариации
    Matrix FT(3,3);
    matrixOps.matrixTranspose(F, FT);
    Matrix temp(3,3);
    matrixOps.matrixMultiply(F, covariance_matrix, temp);
    matrixOps.matrixMultiply(temp, FT, covariance_matrix);
    matrixOps.matrixAdd(covariance_matrix, motion_noise, covariance_matrix);
    delete[] temp.getMatrix(); 
    delete[] F.getMatrix(); 
    delete[] FT.getMatrix(); 
}

void EKFslam::update(double measurement[2]) {
    // Матрица Якоби для модели наблюдения (H)

    const double theta = state[2];
    const double x = state[0], y = state[1];
    const double r_squared = x*x + y*y;

    Matrix H(2,3);
    H(0,0) = -std::cos(theta);
    H(0,1) = -std::sin(theta);
    H(1,0) = std::sin(theta) / r_squared;
    H(1,1) = -std::cos(theta) / r_squared;
    H(1,2) = -1;
    double z_pred[2];
    // Предсказанное измерение
    z_pred[0] = std::sqrt(r_squared);         // Предсказанное расстояние
    z_pred[1] = std::atan2(y, x) - theta;     // Предсказанный угол

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
    if (std::abs(det) < 1e-10) {
        std::cerr << "Матрица S вырождена!" << "(det = " << det << ")"  << std::endl;
        return;
    }
    S_inv(0,0) = S(1,1) / det;
    S_inv(0,1) = -S(0,1) / det;
    S_inv(1,0) = -S(1,0) / det;
    S_inv(1,1) = S(0,0) / det;

    Matrix K(3,2);
    matrixOps.matrixMultiply(covariance_matrix, HT, temp);
    matrixOps.matrixMultiply(temp, S_inv, K);

    // Коррекция состояния
    double dz[2] = {measurement[0] - z_pred[0], measurement[1] - z_pred[1]};
    for (int i = 0; i < STATE_SIZE; ++i) {
        state[i] += K(i,0) * dz[0] + K(i,1) * dz[1];
    }

    // Коррекция ковариации
    Matrix I(3,3);
    Matrix KH(3,3);
    Matrix temp2(3,3);
    matrixOps.matrixMultiply(K, H, KH);
    matrixOps.matrixSubtract(I, KH, temp);
    matrixOps.matrixMultiply(temp2, covariance_matrix, covariance_matrix);
    state[2] = std::fmod(state[2] + M_PI, 2*M_PI) - M_PI;
    delete [] KH.getMatrix();
    delete [] I.getMatrix();
    delete [] temp.getMatrix();
    delete [] temp2.getMatrix();
    delete [] K.getMatrix();
    delete [] S_inv.getMatrix();
    delete [] S.getMatrix();
    delete [] H.getMatrix();
}