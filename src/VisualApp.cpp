#include <vector>
#include <algorithm>
#include <list>
#include <iostream>
#include "VisualApp.hpp"

VisualApp::VisualApp() {
    running = true;
    Surf_Display = NULL;
    window = NULL;
    renderer = NULL;
    WINDOW_WIDTH = 720;
    WINDOW_HEIGHT = 1280;
    POINT_SIZE = 30;
    MOVE_SPEED = 1;
    pointX = 0;
    pointY = 0;
}


 
void VisualApp::OnEvent(SDL_Event* Event) {
    if(Event->type == SDL_QUIT) {
        running = false;
    }
}

bool VisualApp::OnInit() {
    std::cout << "LOG Created surf" << SDL_Init(SDL_INIT_VIDEO);
   
    if(SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cout << "Error init: " << SDL_GetError()  << std::endl;
		system("pause");
        return false;
    }
    

    window = SDL_CreateWindow( "Example", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_HEIGHT, WINDOW_WIDTH, SDL_WINDOW_SHOWN );
    if( !window) {
        std::cout << "Error creating window: " << SDL_GetError()  << std::endl;
		system("pause");
		// End the program
		return 1;
    }
    std::cout << "LOG Created Window";



    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "Ошибка создания рендерера: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    return true;
}

int VisualApp::OnExecute() {
    if(VisualApp::OnInit() == false) {
        return -1;
    }
    SDL_Event Event;
    pointX = WINDOW_WIDTH / 2 - POINT_SIZE / 2;
    pointY = WINDOW_HEIGHT / 2 - POINT_SIZE / 2;
    while(running) {
        while(SDL_PollEvent(&Event)) {
            VisualApp::OnEvent(&Event);
        }
 
        VisualApp::OnLoop();
        VisualApp::OnRender();
    }
 
    VisualApp::OnCleanup();
 
    return 0;
}
void VisualApp::OnLoop(){
    // Обработка нажатий клавиш
    const Uint8* keys = SDL_GetKeyboardState(NULL);
    if (keys[SDL_SCANCODE_W]) pointY -= MOVE_SPEED;  // Движение вверх
    if (keys[SDL_SCANCODE_S]) pointY += MOVE_SPEED;  // Движение вниз
    if (keys[SDL_SCANCODE_A]) pointX -= MOVE_SPEED;  // Движение влево
    if (keys[SDL_SCANCODE_D]) pointX += MOVE_SPEED;  // Движение вправо
    // Ограничение перемещения точки в пределах окна
    if (pointX < 0) pointX = 0;
    if (pointY < 0) pointY = 0;
    if (pointX > WINDOW_WIDTH - POINT_SIZE) pointX = WINDOW_WIDTH - POINT_SIZE;
    if (pointY > WINDOW_HEIGHT - POINT_SIZE) pointY = WINDOW_HEIGHT - POINT_SIZE;
    std::cout << pointX << std::endl;
    std::cout << pointY << std::endl;


};

void VisualApp::OnRender(){
    // Очистка экрана
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);  // Черный цвет
    SDL_RenderClear(renderer);

    // Отрисовка точки (квадрата)
    SDL_Rect pointRect = {pointX, pointY, POINT_SIZE, POINT_SIZE};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);  // Красный цвет
    SDL_RenderFillRect(renderer, &pointRect);

    // Обновление экрана
    SDL_RenderPresent(renderer);
    SDL_Delay(16);  // ~60 FPS
};
void VisualApp::OnCleanup() {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}