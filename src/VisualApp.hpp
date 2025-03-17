#include <SDL2/SDL.h>
 
class VisualApp {
    private:
        bool running;
        SDL_Surface* Surf_Display;
        SDL_Window* window;
 
    public:
        VisualApp();
 
        int OnExecute();
 
    public:
 
        bool OnInit();
 
        void OnEvent(SDL_Event* Event);
 
        void OnLoop();
 
        void OnRender();
 
        void OnCleanup();
};
 