
#include "VisualApp.hpp"


VisualApp::VisualApp() {
    running = true;
    Surf_Display = NULL;
    window = NULL;
    renderer = NULL;
    WINDOW_WIDTH = 720;
    WINDOW_HEIGHT = 1280;
    POINT_SIZE = 15;
    MOVE_SPEED = 1;
    virtual_pos_X = 100;
    virtual_pos_Y = 100;
    slam_virtual_pos_X = 0;
    slam_virtual_pos_Y = 0;
    slam_rotation = 0;
    pointX = 0;
    pointY = 0;
    rotation = 0.0;
    rotationSpeed = 0.5;
    trail;
    slam_trail;
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
    

    window = SDL_CreateWindow( "SLAM", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_HEIGHT, WINDOW_WIDTH, SDL_WINDOW_SHOWN );
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

int VisualApp::OnExecute(EKFslam* slam_obj) {
    if(VisualApp::OnInit() == false) {
        return -1;
    }
    SDL_Event Event;
    pointX = WINDOW_WIDTH / 2 - POINT_SIZE / 2;
    pointY = WINDOW_HEIGHT / 2 - POINT_SIZE / 2;
    int count = 0;
    while(running) {
        while(SDL_PollEvent(&Event)) {
            VisualApp::OnEvent(&Event);
        }

        VisualApp::OnLoop(slam_obj);
        VisualApp::OnRender();
        count++;
        // if (count>30){
        //     running = false;
        // }
        
    }
 
    VisualApp::OnCleanup();
 
    return 0;
}
void VisualApp::OnLoop(EKFslam* slam_obj){
    // Обработка нажатий клавиш
    const Uint8* keys = SDL_GetKeyboardState(NULL);

    double x_move_offset = cos(rotation * M_PI / 180.0)*MOVE_SPEED;
    double y_move_offset = sin(rotation * M_PI / 180.0)*MOVE_SPEED;
    

    // 2. Улучшенные измерения (добавить проверку деления на ноль)
    
    double control[2] = {0,0};
    if (keys[SDL_SCANCODE_W]) {
        virtual_pos_X-=x_move_offset;
        virtual_pos_Y-=y_move_offset;
        control[0] = -MOVE_SPEED;
    }; // Движение вверх
    if (keys[SDL_SCANCODE_S]) {
        virtual_pos_X+=x_move_offset;
        virtual_pos_Y+=y_move_offset;
        control[0] = MOVE_SPEED;
    };  // Движение вниз
    if (keys[SDL_SCANCODE_A]){
        rotation += rotationSpeed;
        control[1] = rotationSpeed*M_PI / 180.0;
    } 
    if (keys[SDL_SCANCODE_D]){
        rotation -= rotationSpeed;
        control[1] = -rotationSpeed*M_PI / 180.0;
    } 
    // std::cout << virtual_pos_X << std::endl;
    // std::cout << virtual_pos_Y << std::endl;
    // std::cout << rotation << std::endl;
    // std::cout << x_move_offset << std::endl;
    // std::cout << y_move_offset << std::endl;
    // std::cout << pointX << std::endl;
    // std::cout << pointY << std::endl;
    // std::cout << slam_obj << std::endl;
    std::cout << "START MEASURMENT" << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << control[0] << std::endl;
    std::cout << control[1] << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!" << std::endl;
    rotation = std::fmod(rotation, 360.0);
    if (rotation > 180.0) rotation -= 360.0;
    if (rotation < -180.0) rotation += 360.0;
    double distance = std::hypot(virtual_pos_X, virtual_pos_Y);
    if (distance < 1e-10) distance = 1e-10;
    
    //double angle = std::atan2(virtual_pos_Y, virtual_pos_X)

    slam_obj->predict(control);
    double measurements[2] = {distance,rotation*M_PI / 180.0};
    std::cout << "=================" << std::endl;
    std::cout << measurements[0] << std::endl;
    std::cout << measurements[1] << std::endl;
    std::cout << "=================" << std::endl;
    // if (!std::isnan(measurements[0]) && !std::isnan(measurements[1])){
    //     slam_obj->update(measurements);
    // }
    

    slam_virtual_pos_X = slam_obj->state[0];
    slam_virtual_pos_Y = slam_obj->state[1];
    slam_rotation = slam_obj->state[2];
    std::cout << virtual_pos_X << std::endl;
    std::cout << virtual_pos_Y << std::endl;
    
    std::cout << rotation << std::endl;
    std::cout << x_move_offset << std::endl;
    std::cout << y_move_offset << std::endl;
    std::cout << pointX << std::endl;
    std::cout << pointY << std::endl;
    std::cout << slam_virtual_pos_X << std::endl;
    std::cout << slam_virtual_pos_Y << std::endl;
    // Ограничение перемещения точки в пределах окна
    if (rotation > 360) rotation -= 360;
    if (rotation < 0) rotation += 360;
    if (pointX < 0) pointX = 0;
    if (pointY < 0) pointY = 0;

    pointX = static_cast<int>(virtual_pos_X);
    pointY = static_cast<int>(virtual_pos_Y);
    // Ограничение перемещения
    
    // pointX = std::max(0, std::min(WINDOW_HEIGHT - POINT_SIZE, pointX));
    // pointX = std::max(0, std::min(WINDOW_WIDTH - POINT_SIZE, pointY));

    // Добавление точки в трек
    if (!trail.empty()) {
        auto& last = trail.back();
        if (last.first != pointX || last.second != pointY) {
            trail.push_back({pointX + POINT_SIZE/2, pointY + POINT_SIZE/2});
        }
    } else {
        trail.push_back({pointX + POINT_SIZE/2, pointY + POINT_SIZE/2});
    };

 
    

};

void VisualApp::OnRender() {
    // Очистка экрана
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // Отрисовка трека
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
    for (size_t i = 1; i < trail.size(); i++) {
        SDL_RenderDrawLine(renderer,
                         trail[i-1].first, trail[i-1].second,
                         trail[i].first, trail[i].second);
    }

    
    
    SDL_Point points[5];
    points[0] = {pointX, pointY};
    points[1] = {pointX + POINT_SIZE,pointY};
    points[2] = {pointX + POINT_SIZE, pointY + POINT_SIZE};
    points[3] = {pointX, pointY + POINT_SIZE};
    points[4] = {pointX, pointY}; // Замыкаем контур
    
    // Создаем прямоугольник для отрисовки

    // Параметры вращения
    SDL_Point center = {pointX + POINT_SIZE/2, pointY + POINT_SIZE/2};

    for(int i = 0; i < 5; i++) {
        points[i] = VisualApp::rotate_point(points[i], center, rotation);
    };
    SDL_RenderDrawLines(renderer,points,5);
    
    // 1. Рисуем заполненный прямоугольник
    
    // 2. Рисуем повернутые границы
    //SDL_RenderDrawRectEx(renderer, &rect, rotation, &center);
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

    int slam_pointX = static_cast<int>(slam_virtual_pos_X);
    int slam_pointY = static_cast<int>(slam_virtual_pos_Y);

    SDL_Point points2[5];
    points2[0] = {slam_pointX, slam_pointY};
    points2[1] = {slam_pointX + POINT_SIZE,slam_pointY};
    points2[2] = {slam_pointX + POINT_SIZE, slam_pointY + POINT_SIZE};
    points2[3] = {slam_pointX, slam_pointY + POINT_SIZE};
    points2[4] = {slam_pointX, slam_pointY}; // Замыкаем контур

    SDL_Point center2 = {slam_pointX + POINT_SIZE/2, slam_pointY + POINT_SIZE/2};

    for(int i = 0; i < 5; i++) {
        points[i] = VisualApp::rotate_point(points2[i], center2, rotation);
    };
    SDL_RenderDrawLines(renderer,points2,5);
    // Обновление экрана
    SDL_RenderPresent(renderer);
    SDL_Delay(16);
}

// Функция поворота точки
SDL_Point VisualApp::rotate_point(SDL_Point point, SDL_Point center, double angle) {
    double rad = angle * M_PI / 180.0;
    
    // Смещаем точку относительно центра вращения
    double translatedX = point.x - center.x;
    double translatedY = point.y - center.y;
    
    // Применяем матрицу поворота
    double cosA = cos(rad);
    double sinA = sin(rad);
    
    double rotatedX = translatedX * cosA - translatedY * sinA;
    double rotatedY = translatedX * sinA + translatedY * cosA;
    //points after move
    // std::cout << "==================" << std::endl;
    // std::cout << rotatedX << std::endl;
    // std::cout << rotatedY << std::endl;
    // std::cout << "==================" << std::endl;
    
    // Возвращаем точку в исходную систему координат
    return SDL_Point{
        static_cast<int>(rotatedX + center.x),
        static_cast<int>(rotatedY + center.y)
    };
}



void VisualApp::OnCleanup() {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}