#include <SDL.h>
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
#include "Logger.hpp"

class VisualApp {
    private:
        bool running;
        SDL_Surface* Surf_Display;
        SDL_Window* window;
        SDL_Renderer* renderer;
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
        std::vector<std::pair<int,int>> trail;
        std::vector<std::pair<int,int>> slam_trail;
        std::vector<std::pair<int,int>> noise_trail;
        std::vector<std::pair<int,int>> landmarks;
        std::vector<std::pair<int,int>>  landmarks_slam;
 
    public:
        VisualApp();
 
        int OnExecute(EKFslam* slam_obj);
 
        bool OnInit();
 
        void OnEvent(SDL_Event* Event);
 
        void OnLoop(EKFslam* slam_obj);
 
        void OnRender();

        SDL_Point rotate_point(SDL_Point point, SDL_Point center, double angle);

        void OnCleanup();
};
 