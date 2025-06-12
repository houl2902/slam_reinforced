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


class App {
    private:
        bool running;
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
        Logger* logger;
        std::vector<std::pair<int,int>> landmarks;
        std::vector<std::pair<int,int>>  landmarks_slam;
        std::vector<Pose*> history_poses_struct;
        std::ifstream input_file;
        
 
    public:
        App();
 
        int OnExecute(EKFslam* slam_obj, GraphSLAM* graph_slam_obj);
 
        bool OnInit();
 
        void OnLoop(EKFslam* slam_obj,GraphSLAM* graph_slam_obj);

        void OnCleanup();
};
