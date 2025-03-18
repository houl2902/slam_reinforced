#include "MatrixFunctions.hpp"
#include "EKFslam.hpp"
#include <cmath>
EKFslam::EKFslam() {
    // Инициализация состояния (робот в начале координат)
    state[0] = 0.0; // x
    state[1] = 0.0; // y
    state[2] = 0.0; // theta

    // Инициализация ковариационной матрицы
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            covariance[i][j] = (i == j) ? 0.1 : 0.0; // Диагональная матрица
        }
    }

    // Инициализация шумов
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            motion_noise[i][j] = (i == j) ? 0.01 : 0.0; // Диагональная матрица
        }
    }

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            measurement_noise[i][j] = (i == j) ? 0.1 : 0.0; // Диагональная матрица
        }
    }
};

void EKFslam::predict(double control[2]) {
    // Модель движения робота
    double dt = 1.0; // Временной шаг
    double v = control[0]; // Линейная скорость
    double w = control[1]; // Угловая скорость

    // Обновление состояния
    state[0] += v * cos(state[2]) * dt; // x
    state[1] += v * sin(state[2]) * dt; // y
    state[2] += w * dt; // theta

    // Матрица Якоби для модели движения (F)
    double F[STATE_SIZE][STATE_SIZE] = {0};
    F[0][0] = 1;
    F[0][2] = -v * sin(state[2]) * dt;
    F[1][1] = 1;
    F[1][2] = v * cos(state[2]) * dt;
    F[2][2] = 1;

    // Обновление ковариации
    double FT[STATE_SIZE][STATE_SIZE];
    MatrixFunctions::matrixTranspose(F, FT);

    double temp[STATE_SIZE][STATE_SIZE];
    MatrixFunctions::matrixMultiply(F, covariance, temp);
    MatrixFunctions::matrixMultiply(temp, FT, covariance);
    MatrixFunctions::matrixAdd(covariance, motion_noise, covariance);
};

void EKFSLAM::update(double measurement[2]) {
    // Матрица Якоби для модели наблюдения (H)
    double H[2][STATE_SIZE] = {0};
    H[0][0] = -cos(state[2]);
    H[0][1] = -sin(state[2]);
    H[1][0] = sin(state[2]) / (state[0] * state[0] + state[1] * state[1]);
    H[1][1] = -cos(state[2]) / (state[0] * state[0] + state[1] * state[1]);

    // Предсказанное измерение
    double z_pred[2] = {0.1,2};

    // Матрица усиления Калмана (K)
    double HT[STATE_SIZE][2];
    MatrixFunctions::matrixTranspose(H, HT);

    double S[2][2];
    MatrixFunctions::matrixMultiply(H, covariance, temp);
    MatrixFunctions::matrixMultiply(temp, HT, S);
    MatrixFunctions::matrixAdd(S, measurement_noise, S);

    double S_inv[2][2];
    // Вычисление обратной матрицы S (для простоты используем явную формулу для 2x2)
    double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (det == 0) {
        std::cerr << "Матрица S вырождена!" << std::endl;
        return;
    }
    S_inv[0][0] = S[1][1] / det;
    S_inv[0][1] = -S[0][1] / det;
    S_inv[1][0] = -S[1][0] / det;
    S_inv[1][1] = S[0][0] / det;

    double K[STATE_SIZE][2];
    MatrixFunctions::matrixMultiply(covariance, HT, temp);
    MatrixFunctions::matrixMultiply(temp, S_inv, K);

    // Коррекция состояния
    double dz[2] = {measurement[0] - z_pred[0], measurement[1] - z_pred[1]};
    for (int i = 0; i < STATE_SIZE; ++i) {
        state[i] += K[i][0] * dz[0] + K[i][1] * dz[1];
    }

    // Коррекция ковариации
    double I[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; ++i) {
        I[i][i] = 1;
    }

    double KH[STATE_SIZE][STATE_SIZE];
    MatrixFunctions::matrixMultiply(K, H, KH);
    MatrixFunctions::matrixSubtract(I, KH, temp);
    MatrixFunctions::matrixMultiply(temp, covariance, covariance);
};