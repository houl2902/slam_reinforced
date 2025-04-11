#include <gtest/gtest.h>
#include "/home/houl/slam_reinforced/src/EKFslam.hpp"
//#include "/home/houl/slam_reinforced/src/MatrixFunctions.hpp"
#include <cmath>

#define GTEST_BOX "[     cout ] "
class MatrixOperationsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            // Инициализация фильтра перед каждым тестом
            ops = std::make_unique<MatrixOperations>();
        }
    
        std::unique_ptr<MatrixOperations> ops;
    };

class EKFslamTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Инициализация фильтра перед каждым тестом
        ekf = std::make_unique<EKFslam>();
    }

    std::unique_ptr<EKFslam> ekf;
};

    
TEST_F(MatrixOperationsTest, MultiplyValid) {
    Matrix A(2, 3), B(3, 2), result(2, 2);
    
    // Заполняем матрицы тестовыми данными
    A(0,0)=1; A(0,1)=2; A(0,2)=3;
    A(1,0)=4; A(1,1)=5; A(1,2)=6;
    
    B(0,0)=7; B(0,1)=8;
    B(1,0)=9; B(1,1)=10;
    B(2,0)=11; B(2,1)=12;
        
    ops->matrixMultiply(A, B, result);
        
        // Проверяем результат
    EXPECT_NEAR(result(0,0), 58, 1e-9);  // 1*7 + 2*9 + 3*11
    EXPECT_NEAR(result(0,1), 64, 1e-9);
    EXPECT_NEAR(result(1,0), 139, 1e-9);
    EXPECT_NEAR(result(1,1), 154, 1e-9);
}
    
TEST_F(MatrixOperationsTest, MultiplyInvalidDimensions) {
        Matrix A(2, 2), B(3, 2), result(2, 2);
        EXPECT_THROW(ops->matrixMultiply(A, B, result), std::invalid_argument);
}
    
TEST_F(MatrixOperationsTest, MultiplyWithNaN) {
        Matrix A(1, 1), B(1, 1), result(1, 1);
        A(0,0) = NAN;
        B(0,0) = 2.0;
        
        testing::internal::CaptureStderr();
        ops->matrixMultiply(A, B, result);
        std::string output = testing::internal::GetCapturedStderr();
        
        EXPECT_TRUE(output.find("Warning") != std::string::npos);
        EXPECT_EQ(result(0,0), 1e-10);
}
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
    std::cout << GTEST_BOX << ekf->state[0] << std::endl;
    std::cout << GTEST_BOX << ekf->state[1] << std::endl;
    std::cout << GTEST_BOX << ekf->state[2] << std::endl;
    EXPECT_NEAR(ekf->state[0], 100.0 + std::cos(0.0001), 1e-5);
    EXPECT_NEAR(ekf->state[1], 100.0 + std::sin(0.0001), 1e-5);
    EXPECT_NEAR(ekf->state[2], 0.0001, 1e-5);
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

TEST_F(EKFslamTest, UpdateWithPerfectMeasurement) {
    // Инициализируем робота в (100, 100) и добавляем landmark в (110, 100)
    ekf->covariance_matrix(0,0) = 1.0; 
    ekf->state[0] = 100.0;
    ekf->state[1] = 100.0;
    ekf->state[2] = 0.0;
    ekf->addLandmark(110.0, 100.0); // landmark_id = 0
    
    // Имитируем идеальное измерение (расстояние 10, угол 0)
    double measurement[2] = {10.0, 0.0};
    ekf->update(measurement, 0);
    
    // Проверяем что состояние робота не изменилось (идеальное измерение)
    EXPECT_NEAR(ekf->state[0], 100.0, 1e-5);
    EXPECT_NEAR(ekf->state[1], 100.0, 1e-5);
    EXPECT_NEAR(ekf->state[2], 0.0, 1e-5);
    
    // Проверяем что ковариация уменьшилась
    EXPECT_LT(ekf->covariance_matrix(0,0), 1); // Изначально было 0.1
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}