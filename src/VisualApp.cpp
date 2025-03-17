#include <vector>
#include <algorithm>
#include <list>
#include <iostream>
#include "VisualApp.hpp"

VisualApp::VisualApp() {
    running = true;
    Surf_Display = NULL;
    window = NULL;
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
    std::cout << "LOG Created surf";
    window = SDL_CreateWindow( "Example", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1280, 720, SDL_WINDOW_SHOWN );


    if( !window) {
        std::cout << "Error creating window: " << SDL_GetError()  << std::endl;
		system("pause");
		// End the program
		return 1;
    }
    Surf_Display = SDL_GetWindowSurface( window );

    if( !Surf_Display ) {
        std::cout << "Error creating window: " << SDL_GetError()  << std::endl;
		system("pause");
		// End the program
		return 1;
    }

    return true;
}

int VisualApp::OnExecute() {
    if(VisualApp::OnInit() == false) {
        return -1;
    }
 
    SDL_Event Event;
 
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

};

void VisualApp::OnRender(){

};
void VisualApp::OnCleanup() {
    SDL_Quit();
}