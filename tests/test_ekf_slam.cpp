#include <gtest/gtest.h>
#include "/home/houl/slam_reinforced/src/EKFslam.hpp"
//#include "/home/houl/slam_reinforced/src/MatrixFunctions.hpp"
#include "/home/houl/slam_reinforced/src/GraphSLAM.hpp"
#include <cmath>
#include <random>

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

class GraphSLAMTest : public ::testing::Test {
    protected:
        void SetUp() override {
            slam = new GraphSLAM();
            
            // Инициализация параметров шумов
            slam->sigma_odom_x = 0.1;
            slam->sigma_odom_y = 0.1;
            slam->sigma_odom_theta = 0.05;
            slam->sigma_obs_range = 0.1;
            slam->sigma_obs_bearing = 0.05;
            
            // Генератор случайных чисел
            rng.seed(42);
        }
    
        void TearDown() override {
            delete slam;
        }
    
        void generateStraightPath(int num_poses, double step_size) {
            for (int i = 0; i < num_poses; ++i) {
                double pos[3] = {i * step_size, 0.0, 0.0};
                double measurement[2] = {0}; // Пустые измерения
                
                if (i > 0 && i % 3 == 0) {
                    // Каждые 3 шага добавляем измерение ориентира
                    measurement[0] = 5.0; // range
                    measurement[1] = 0.0; // bearing
                }
                
                slam->addPose(pos, measurement);
            }
        }
    
        void generateLoopPath(int num_poses, double radius) {
            for (int i = 0; i < num_poses; ++i) {
                double angle = 2 * M_PI * i / num_poses;
                double pos[3] = {radius * cos(angle), radius * sin(angle), angle + M_PI/2};
                
                double measurement[2] = {0};
                if (i > 0 && i % 3 == 0) {
                    // Измерение центрального ориентира
                    double dx = -pos[0];
                    double dy = -pos[1];
                    measurement[0] = sqrt(dx*dx + dy*dy);
                    measurement[1] = atan2(dy, dx) - pos[2];
                }
                
                slam->addPose(pos, measurement);
            }
        }
    
        double calculatePositionError() {
            double total_error = 0.0;
            int count = 0;
            
            for (size_t i = 0; i < slam->history_poses_struct.size(); ++i) {
                Pose* pose = slam->history_poses_struct[i];
                double expected_x = i * 1.0; // Для прямого пути
                double expected_y = 0.0;
                
                double dx = pose->x - expected_x;
                double dy = pose->y - expected_y;
                total_error += sqrt(dx*dx + dy*dy);
                count++;
            }
            
            return count > 0 ? total_error / count : 0.0;
        }
    
        GraphSLAM* slam;
        std::mt19937 rng;
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

TEST_F(EKFslamTest, UpdateMultipleLandmarks) {
    ekf->state[0] = 0.0;
    ekf->state[1] = 0.0;
    ekf->state[2] = 0.0;
    ekf->addLandmark(10.0, 0.0);  // landmark 0
    ekf->addLandmark(0.0, 10.0);  // landmark 1
    
    // Обновляем первый landmark
    double meas1[2] = {10.1, 0.0};
    ekf->update(meas1, 0);
    
    // Обновляем второй landmark
    double meas2[2] = {9.9, M_PI/2};
    ekf->update(meas2, 1);
    
    // Проверки
    EXPECT_NEAR(ekf->state[3], 10.0, 0.2);  // landmark 0 x
    EXPECT_NEAR(ekf->state[5], 0.0, 0.2);   // landmark 1 x
    EXPECT_NEAR(ekf->state[6], 10.0, 0.2);  // landmark 1 y
}

// TEST_F(GraphSLAMTest, ConvergenceTest) {
//     // Добавляем узлы и ребра в граф
//     for (size_t i = 0; i < true_poses.size(); ++i) {
//         slam->addPoseNode(i, i == 0 ? true_poses[0] : Pose{0,0,0});
//     }

//     for (size_t i = 1; i < true_poses.size(); ++i) {
//         slam->addOdometryEdge(i-1, i, odometry_measurements[i-1]);
//     }

//     for (size_t pose_idx = 0; pose_idx < true_poses.size(); ++pose_idx) {
//         for (const auto& lm : landmarks) {
//             if (pose_idx % 3 == 0) { // Не на каждом шаге измеряем ориентиры
//                 auto rel_pos = vector_subtract(lm, true_poses[pose_idx]);
//                 slam->addLandmarkObservation(pose_idx, lm, rel_pos);
//             }
//         }
//     }

//     // Запускаем оптимизацию и проверяем сходимость
//     std::vector<double> errors;
//     for (int iter = 0; iter < 20; ++iter) {
//         slam->optimize(1); // 1 итерация за раз
//         auto estimated = slam->optimizeGraph();
//         double error = calculatePositionError(estimated);
//         errors.push_back(error);
        
//         // Для отладки можно выводить ошибку на каждой итерации
//         // std::cout << "Iteration " << iter << ", error: " << error << std::endl;
//     }

//     // Проверяем, что ошибка уменьшается
//     for (size_t i = 1; i < errors.size(); ++i) {
//         EXPECT_LE(errors[i], errors[i-1] * 1.1); // Допускаем небольшой рост из-за численных ошибок
//     }

//     // Проверяем, что финальная ошибка мала
//     EXPECT_LT(errors.back(), 0.5);
// }







// TEST_F(GraphSLAMTest, StraightPathConvergence) {
//     // 1. Генерация прямого пути
//     generateStraightPath(10, 1.0);
    
//     // 2. Проверка начальной ошибки
//     double initial_error = calculatePositionError();
//     std::cout << "Initial error: " << initial_error << std::endl;
    
//     // 3. Оптимизация
//     std::vector<double> errors;
//     for (int iter = 0; iter < 5; ++iter) {
//         slam->optimizeGraph(1);
//         double error = calculatePositionError();
//         errors.push_back(error);
//         std::cout << "Iteration " << iter << ", error: " << error << std::endl;
//     }
    
//     // 4. Проверка сходимости
//     for (size_t i = 1; i < errors.size(); ++i) {
//         EXPECT_LE(errors[i], errors[i-1] * 1.1); // Допускаем небольшой рост
//     }
    
//     // 5. Проверка финальной ошибки
//     EXPECT_LT(errors.back(), 0.5);
    
//     // 6. Проверка количества ориентиров
//     EXPECT_GT(slam->landmarks.size(), 0);
// }

// TEST_F(GraphSLAMTest, LoopClosureDetection) {
//     // 1. Генерация петли
//     generateLoopPath(20, 5.0);
    
//     // 2. Проверка начального состояния
//     EXPECT_GT(slam->history_poses_struct.size(), 10);
    
//     // 3. Оптимизация
//     slam->optimizeGraph(5);
    
//     // 4. Проверка ограничений замыкания цикла
//     bool has_loop_closures = false;
//     for (auto& constraint : slam->odometry_constraints) {
//         if (abs(constraint->from_pose_id - constraint->to_pose_id) > 5) {
//             has_loop_closures = true;
//             break;
//         }
//     }
//     EXPECT_TRUE(has_loop_closures);
    
//     // 5. Проверка ошибки
//     double final_error = 0.0;
//     for (size_t i = 0; i < slam->history_poses_struct.size(); ++i) {
//         Pose* pose = slam->history_poses_struct[i];
//         double angle = 2 * M_PI * i / slam->history_poses_struct.size();
//         double expected_x = 5.0 * cos(angle);
//         double expected_y = 5.0 * sin(angle);
        
//         double dx = pose->x - expected_x;
//         double dy = pose->y - expected_y;
//         final_error += sqrt(dx*dx + dy*dy);
//     }
//     final_error /= slam->history_poses_struct.size();
    
//     EXPECT_LT(final_error, 1.0);
// }

// TEST_F(GraphSLAMTest, LandmarkInitialization) {
//     // 1. Добавление поз с измерениями ориентиров
//     for (int i = 0; i < 5; ++i) {
//         double pos[3] = {i * 1.0, 0.0, 0.0};
//         double measurement[2] = {5.0, 0.0}; // Ориентир прямо перед роботом
//         slam->addPose(pos, measurement);
//     }
    
//     // 2. Проверка инициализации ориентиров
//     EXPECT_EQ(slam->landmarks.size(), 1);
    
//     // 3. Проверка координат ориентира
//     Landmark* lm = slam->landmarks[0];
//     EXPECT_NEAR(lm->x, 5.0, 0.1);
//     EXPECT_NEAR(lm->y, 0.0, 0.1);
    
//     // 4. Проверка наблюдений
//     EXPECT_EQ(slam->observations.size(), 5);
// }

TEST_F(GraphSLAMTest, OptimizationReducesError) {
    // 1. Генерация пути с шумом

    for (int i = 0; i < 10; ++i) {
        double pos[3] = {i * 1 + 0.1*(rng()/(double)RAND_MAX-0.5), 
                        0.1*(rng()/(double)RAND_MAX-0.5), 
                        0.1*(rng()/(double)RAND_MAX-0.5)};
        double measurement[2] = {0};
        
        if (i % 3 == 0) {
            measurement[0] = 5 + 0.2*(rng()/(double)RAND_MAX-0.5);
            measurement[1] = 0.1*(rng()/(double)RAND_MAX-0.5);
        }
        
        slam->addPose(pos, measurement);
    }
    std::cout << "Initial error: " << "HERE" << ", Final error: " << "final_error" << std::endl;
    // 2. Измерение начальной ошибки
    double initial_error = calculatePositionError();
    
    // 3. Оптимизация
    slam->optimizeGraph(20);
    
    // 4. Проверка уменьшения ошибки
    double final_error = calculatePositionError();
    std::cout << "Initial error: " << initial_error << ", Final error: " << final_error << std::endl;
};

 TEST_F(GraphSLAMTest, TwoPosesOneOdometryConstraintSimpleTranslation) {
    double new_pose1[3] = {0.0, 0.0,0.0};
    double m1[2] = {0,0};
    slam->addPose(new_pose1,m1); 
    double new_pose2[3] = {0.9, 0.1,0.05};
    double m2[2] = {0,0};
    slam->addPose(new_pose2,m2);     
    slam->addOdometryConstraint(new OdometryConstraint(0, 1, 1.0, 0.0, 0.0)); 

    slam->sigma_odom_x = 0.1;
    slam->sigma_odom_y = 0.1;
    slam->sigma_odom_theta = 0.05;

    slam->optimizeGraph(30); 
    //std::string output = GetCapturedStdout();

    //EXPECT_NE(output.find("Сходимость достигнута."), std::string::npos) << "Output: " << output;

    Pose* p0 = slam->getPoseByIndex(0);
    Pose* p1 = slam->getPoseByIndex(1);

    ASSERT_NE(p0, nullptr);
    ASSERT_NE(p1, nullptr);

    EXPECT_NEAR(p0->x, 0.0, 1e-5); 
    EXPECT_NEAR(p0->y, 0.0, 1e-5);
    EXPECT_NEAR(p0->theta, 0.0, 1e-5);

    // CG should be more accurate, so tolerances can be tighter if the system is well-conditioned
    EXPECT_NEAR(p1->x, 1.0, 1e-3); 
    EXPECT_NEAR(p1->y, 0.0, 1e-3);
    EXPECT_NEAR(p1->theta, 0.0, 1e-3); 
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}