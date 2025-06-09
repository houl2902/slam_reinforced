#include <vector>
#include <algorithm>
#include <list>
#include <iostream>
#include <functional>
#include <utility>
#include <cmath>
#include <array>
#include <format>
#include <string>
#include "EKFslam.hpp"
#include "GraphSLAM.hpp"
#include "Logger.hpp"
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

class ParrallelApp {
    private:
        int WINDOW_WIDTH;
        int WINDOW_HEIGHT;
        int POINT_SIZE;
        int MOVE_SPEED;
        int pointX;
        int pointY;
        double virtual_pos_X;
        double virtual_pos_Y;
        float rotation;
        float rotationSpeed;
        double slam_virtual_pos_X;
        double slam_virtual_pos_Y;
        double noisePointX;
        double noisePointY;
        double noise_rotation;
        float slam_rotation;
        double control[2];
        Logger* logger;
        std::vector<std::pair<int,int>> landmarks;
        std::vector<std::pair<int,int>>  landmarks_slam;
        std::vector<Pose*> history_poses_struct;
        std::ifstream input_file;
        std::mutex data_mutex; // Для синхронизации доступа к общим данным
        std::atomic<bool> running; // Атомарный флаг для управления потоками
        std::thread ekf_thread;
        std::thread graph_thread;
 
    public:
        ParrallelApp();
 
        int OnExecute(EKFslam* slam_obj, GraphSLAM* graph_slam_obj);
        
        void EKFThreadFunc(EKFslam* slam_obj);

        void GraphThreadFunc(GraphSLAM* graph_slam_obj, EKFslam* slam_obj);
 
        bool OnInit();
 
        void OnLoop(EKFslam* slam_obj,GraphSLAM* graph_slam_obj);

        void OnCleanup();
};
 