#include <SDL.h>
#include <vector>
#include <algorithm>
#include <list>
#include <iostream>
#include <functional>
#include <utility>

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
        std::vector<std::pair<int,int>> trail;
 
    public:
        VisualApp();
 
        int OnExecute();
 
        bool OnInit();
 
        void OnEvent(SDL_Event* Event);
 
        void OnLoop();
 
        void OnRender();
 
        void OnCleanup();
};
 