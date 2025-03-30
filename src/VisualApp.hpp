#include <SDL.h>
#include <vector>
#include <algorithm>
#include <list>
#include <iostream>
#include <functional>
#include <utility>
#include <cmath>
#include <array>
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
        std::vector<std::pair<int,int>> trail;
 
    public:
        VisualApp();
 
        int OnExecute();
 
        bool OnInit();
 
        void OnEvent(SDL_Event* Event);
 
        void OnLoop();
 
        void OnRender();

        SDL_Point rotate_point(SDL_Point point, SDL_Point center, double angle);

        int SDL_RenderDrawRectEx(SDL_Renderer* renderer, SDL_Rect* rect, double angle, SDL_Point* center);

        void OnCleanup();
};
 