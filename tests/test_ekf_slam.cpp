#include <gtest/gtest.h>
#include "/home/houl/slam_reinforced/src/EKFslam.hpp"
#include <cmath>

#define GTEST_BOX "[     cout ] "
class EKFslamTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Инициализация фильтра перед каждым тестом
        ekf = std::make_unique<EKFslam>();
    }

    std::unique_ptr<EKFslam> ekf;
};

// Тест инициализации состояния
TEST_F(EKFslamTest, InitialState) {
    EXPECT_NEAR(ekf->state[0], 100.0, 1e-5);  // Проверка начального X
    EXPECT_NEAR(ekf->state[1], 100.0, 1e-5);  // Проверка начального Y
    EXPECT_NEAR(ekf->state[2], 0.0001, 1e-5); // Проверка начального угла
}

// Тест предсказания (движение прямо)
TEST_F(EKFslamTest, PredictStraightMotion) {
    double control[2] = {1.0, 0.0};  // v = 1 м/с, w = 0 рад/с
    ekf->predict(control);

    // Ожидаемые значения после dt = 1 сек
    EXPECT_NEAR(ekf->state[0], 100.0 + std::cos(0.0001), 1e-5);
    EXPECT_NEAR(ekf->state[1], 100.0 + std::sin(0.0001), 1e-5);
    EXPECT_NEAR(ekf->state[2], 0.0001, 1e-5);
    std::cout << GTEST_BOX << ekf->state[0] << std::endl;
    std::cout << GTEST_BOX << ekf->state[1] << std::endl;
    std::cout << GTEST_BOX << ekf->state[2] << std::endl;
}

// Тест предсказания (поворот)
TEST_F(EKFslamTest, PredictRotation) {
    double control[2] = {0.0, M_PI/4};  // v = 0, w = π/4 рад/с
    ekf->predict(control);

    // Ожидаемый угол после dt = 1 сек
    EXPECT_NEAR(ekf->state[2], 0.0001 + M_PI/4, 1e-5);
    std::cout << GTEST_BOX << ekf->state[0] << std::endl;
    std::cout << GTEST_BOX << ekf->state[1] << std::endl;
    std::cout << GTEST_BOX << ekf->state[2] << std::endl;
}

// Тест коррекции (обновление состояния)
// TEST_F(EKFslamTest, UpdateState) {
//     double measurement[2] = {10.0, 0.0};  // Расстояние = 10 м, угол = 0 рад
//     ekf->update(measurement);

//     // Проверка, что состояние изменилось (точные значения зависят от K)
//     EXPECT_TRUE(ekf->state[0] != 100.0 || ekf->state[1] != 100.0);
//     std::cout << GTEST_BOX << ekf->state[0] << std::endl;
//     std::cout << GTEST_BOX << ekf->state[1] << std::endl;
//     std::cout << GTEST_BOX << ekf->state[2] << std::endl;
// }

// Тест обработки вырожденного случая (деление на ноль)
// TEST_F(EKFslamTest, NearZeroDistance) {
//     ekf->state[0] = 0.0;  // Робот в начале координат
//     ekf->state[1] = 0.0;
//     double measurement[2] = {0.0, 0.0};
    
//     // Не должно быть ошибок (например, деления на ноль)
//     EXPECT_NO_THROW(ekf->update(measurement));
//     std::cout << GTEST_BOX << ekf->state[0] << std::endl;
//     std::cout << GTEST_BOX << ekf->state[1] << std::endl;
//     std::cout << GTEST_BOX << ekf->state[2] << std::endl;
// }

// Тест ковариационной матрицы после предсказания
TEST_F(EKFslamTest, CovarianceAfterPredict) {
    double initial_cov = ekf->covariance_matrix(0, 0);
    double control[2] = {1.0, 0.0};
    ekf->predict(control);
    std::cout << GTEST_BOX << initial_cov << std::endl;
    std::cout << GTEST_BOX << ekf->covariance_matrix(0, 0) << std::endl;
    // Ковариация должна увеличиться из-за шума
    EXPECT_GT(ekf->covariance_matrix(0, 0), initial_cov);
}

// Тест устойчивости при больших шумах
TEST_F(EKFslamTest, HighNoiseRobustness) {
    ekf->vel_noise_std = 10.0;  // Огромный шум скорости
    double control[2] = {1.0, 0.0};
    ekf->predict(control);
    std::cout << GTEST_BOX << ekf->state[0] << std::endl;
    std::cout << GTEST_BOX << ekf->state[1] << std::endl;
    std::cout << GTEST_BOX << ekf->state[2] << std::endl;

    // Фильтр не должен "взрываться" (ковариация остаётся конечной)
    EXPECT_LT(ekf->covariance_matrix(0, 0), 1e6);
}

TEST_F(EKFslamTest, CovarianceAfterPredictWithLandMark) {
    double initial_cov = ekf->covariance_matrix(0, 0);
    double control[2] = {1.0, 0.0};
    ekf->addLandmark(10.0,2.0);
    std::cout << GTEST_BOX << ekf->covariance_matrix.getSize()[0] << std::endl;
    std::cout << GTEST_BOX << ekf->covariance_matrix.getSize()[1] << std::endl;
    ekf->predict(control);
    std::cout << GTEST_BOX << initial_cov << std::endl;
    std::cout << GTEST_BOX << ekf->covariance_matrix(0, 0) << std::endl;
    // Ковариация должна увеличиться из-за шума
    EXPECT_GT(ekf->covariance_matrix(0, 0), initial_cov);
}

// TEST_F(EKFslamTest, UpdateWithPerfectMeasurement) {
//     // Исходное состояние: робот в (100, 100), theta = 0
//     ekf->state[0] = 100.0;
//     ekf->state[1] = 100.0;
//     ekf->state[2] = 0.0;

//     // Измерение: расстояние = 10, угол = 0 (робот "видит" landmark в (110, 100))
//     double measurement[2] = {10.0, 0.0};
//     ekf->update(measurement);

//     // Ожидаемое состояние после коррекции (должно приблизиться к (110, 100))
//     EXPECT_NEAR(ekf->state[0], 110.0, 0.1);
//     EXPECT_NEAR(ekf->state[1], 100.0, 0.1);
//     EXPECT_NEAR(ekf->state[2], 0.0, 0.01);
// }

// TEST_F(EKFslamTest, UpdateWithRobotPositionEstimate) {
//     // Исходное состояние фильтра: робот предполагает, что находится в (100, 100)
//     ekf->state[0] = 100.0;  // x
//     ekf->state[1] = 100.0;  // y
//     ekf->state[2] = 0.0;    // theta
    
//     // Настройка ковариации (высокая начальная неопределенность)
//     ekf->covariance_matrix(0,0) = 10.0;  // Неопределенность по x
//     ekf->covariance_matrix(1,1) = 10.0;  // Неопределенность по y
//     ekf->covariance_matrix(2,2) = 1.0;   // Неопределенность по углу

//     // Внешняя оценка положения робота (например, по GPS или другим датчикам)
//     // Считаем, что робот на самом деле находится в (105, 98)
//     double robot_position_estimate[2] = {105.0, 98.0};
    
//     // Обновляем фильтр с этой оценкой
//     ekf->update(robot_position_estimate);

//     // Проверяем, что состояние скорректировалось в сторону оценки
//     EXPECT_GT(ekf->state[0], 100.0) << "X should increase toward 105.0";
//     EXPECT_LT(ekf->state[1], 100.0) << "Y should decrease toward 98.0";
//     EXPECT_NEAR(ekf->state[2], 0.0, 0.01) << "Orientation shouldn't change";

//     // Проверяем уменьшение неопределенности
//     EXPECT_LT(ekf->covariance_matrix(0,0), 10.0) << "X uncertainty should decrease";
//     EXPECT_LT(ekf->covariance_matrix(1,1), 10.0) << "Y uncertainty should decrease";

//     // Проверяем величину коррекции (должны приблизиться к 105, 98)
//     EXPECT_NEAR(ekf->state[0], 105.0, 2.0) << "X should be close to 105.0";
//     EXPECT_NEAR(ekf->state[1], 98.0, 2.0) << "Y should be close to 98.0";
// }

// TEST_F(EKFslamTest, MultipleUpdates) {
//     ekf->state[0] = 0.0;
//     ekf->state[1] = 0.0;
//     ekf->state[2] = 0.0;

//     // Первое измерение: landmark в (5, 0)
//     double measurement1[2] = {5.0, 0.0};
//     ekf->update(measurement1);

//     // Второе измерение: landmark в (0, 5)
//     double measurement2[2] = {5.0, M_PI/2};
//     ekf->update(measurement2);

//     // Робот должен быть ближе к (0, 0)
//     EXPECT_NEAR(ekf->state[0], 0.0, 1.0);
//     EXPECT_NEAR(ekf->state[1], 0.0, 1.0);
// }

// TEST_F(EKFslamTest, CovarianceDecreasesAfterUpdate) {
//     ekf->state[0] = 0.0;
//     ekf->state[1] = 0.0;
//     ekf->state[2] = 0.0;

//     // Увеличиваем неопределённость
//     ekf->covariance_matrix(0,0) = 10.0;
//     ekf->covariance_matrix(1,1) = 10.0;

//     double initial_cov = ekf->covariance_matrix(0,0);
//     double measurement[2] = {5.0, 0.0};  // Точное измерение
//     ekf->update(measurement);

//     // Ковариация должна уменьшиться
//     EXPECT_LT(ekf->covariance_matrix(0,0), initial_cov);
// }

// TEST_F(EKFslamTest, UpdateWithBearingAngle) {
//     ekf->state[0] = 0.0;
//     ekf->state[1] = 0.0;
//     ekf->state[2] = M_PI/4;  // Робот повёрнут на 45 градусов

//     // Измерение: расстояние = 10, угол = π/4 (ожидаемый landmark в (7.07, 7.07))
//     double measurement[2] = {10.0, M_PI/4};
//     ekf->update(measurement);

//     // Проверка координат (с учётом поворота)
//     std::cout << GTEST_BOX << (ekf->state[0]) << std::endl;
//     EXPECT_NEAR(ekf->state[0], 7.07, 0.1);
//     EXPECT_NEAR(ekf->state[1], 7.07, 0.1);
//     EXPECT_NEAR(ekf->state[2], M_PI/4, 0.01);
// }

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}