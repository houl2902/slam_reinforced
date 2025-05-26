
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
    noise_rotation = 0;
    pointX = 0;
    pointY = 0;
    noisePointX = 100;
    noisePointY = 100;
    rotation = 0.0;
    rotationSpeed = 0.5;
    trail;
    slam_trail;
    noise_trail;
    landmarks;
    landmarks_slam;

    history_poses_struct;
    logger = NULL;

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

    logger = new Logger({"greenVector.txt","yellowVector.txt","redVector.txt"});
    return true;
}

int VisualApp::OnExecute(EKFslam* slam_obj, GraphSLAM* graph_slam_obj) {
    if(VisualApp::OnInit() == false) {
        return -1;
    }
    SDL_Event Event;
    pointX = WINDOW_WIDTH / 2 - POINT_SIZE / 2;
    pointY = WINDOW_HEIGHT / 2 - POINT_SIZE / 2;
    landmarks.push_back({300,300});
    slam_obj->addLandmark(300.0,300.0);
    slam_obj->addLandmark(350.0,400.0);
    landmarks.push_back({350,400});
    while(running) {
        while(SDL_PollEvent(&Event)) {
            VisualApp::OnEvent(&Event);
        }
        VisualApp::OnLoop(slam_obj,graph_slam_obj);
        VisualApp::OnRender();
    }
 
    VisualApp::OnCleanup();
 
    return 0;
}
void VisualApp::OnLoop(EKFslam* slam_obj, GraphSLAM* graph_slam_obj){
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

    rotation = std::fmod(rotation, 360.0);
    if (rotation > 180.0) rotation -= 360.0;
    if (rotation < -180.0) rotation += 360.0;
    //double distance = std::hypot(virtual_pos_X, virtual_pos_Y);

   

    double logger_array_green[3] = {virtual_pos_X,virtual_pos_Y,rotation};
    logger->writeMatrixCoords(logger_array_green,"greenVector.txt");
    double distance = sqrt((300-virtual_pos_X) * (300-virtual_pos_X) + (300-virtual_pos_Y) * (300-virtual_pos_Y));
    if (distance < 1e-10) distance = 1e-10;
    double angle = atan2(300-virtual_pos_Y,300-virtual_pos_X) - rotation*M_PI / 180.0;
    //double angle = std::atan2(virtual_pos_Y, virtual_pos_X)

    slam_obj->predict(control);
    
    for (int i = 0; i<landmarks.size(); i++){
      std::cout << "=================" << std::endl;
      double distance = sqrt((landmarks[i].first-virtual_pos_X) * (landmarks[i].first-virtual_pos_X) + (landmarks[i].second-virtual_pos_Y) * (landmarks[i].second-virtual_pos_Y));
      if (distance < 1e-10) distance = 1e-10;
      double angle = atan2(landmarks[i].second-virtual_pos_Y,landmarks[i].first-virtual_pos_X) - rotation*M_PI / 180.0;
      double measurements[2] = {distance,angle};
    //   std::cout << "=================" << std::endl;
    //   std::cout << "MESUREMENT" << std::endl;
    //   std::cout << measurements[0] << std::endl;
    //   std::cout << measurements[1] << std::endl;
    //   std::cout << "=================" << std::endl;
      if (!std::isnan(measurements[0]) && !std::isnan(measurements[1])){
        slam_obj->update(measurements,i);
      }
    //   std::cout << "=================" << std::endl;
    //   std::cout << "currMESUREMENT" << std::endl;
    //   std::cout << slam_obj->curr_measurement[0] << std::endl;
    //   std::cout << slam_obj->curr_measurement[1] << std::endl;
    //   std::cout << "=================" << std::endl;
    }
    //double angle = std::atan2(virtual_pos_Y, virtual_pos_X)
    

    slam_virtual_pos_X = slam_obj->state[0];
    slam_virtual_pos_Y = slam_obj->state[1];
    slam_rotation = slam_obj->state[2];

    graph_slam_obj->addPose(slam_obj->state, slam_obj->curr_measurement);
    history_poses_struct = graph_slam_obj->history_poses_struct;

    logger->writeMatrixCoords(slam_obj->state,"yellowVector.txt");
    
   

    // Ограничение перемещения точки в пределах окна
    if (rotation > 360) rotation -= 360;
    if (rotation < 0) rotation += 360;
    if (pointX < 0) pointX = 0;
    if (pointY < 0) pointY = 0;

    pointX = static_cast<int>(virtual_pos_X);
    pointY = static_cast<int>(virtual_pos_Y);
    landmarks_slam.push_back({static_cast<int>(slam_obj->state[3]),static_cast<int>(slam_obj->state[4])});
    landmarks_slam.push_back({static_cast<int>(slam_obj->state[5]),static_cast<int>(slam_obj->state[6])});
    pointX = static_cast<int>(virtual_pos_X);
    pointY = static_cast<int>(virtual_pos_Y);

    if (graph_slam_obj->odometry_constraints.size() > 3) { // Изменено условие
        graph_slam_obj->optimizeGraph(15);
    }
    
    // Добавление точки в трек
    if (!trail.empty()) {
        auto& last = trail.back();
        if (last.first != pointX || last.second != pointY) {
            trail.push_back({pointX + POINT_SIZE/2, pointY + POINT_SIZE/2});
        }
    } else {
        trail.push_back({pointX + POINT_SIZE/2, pointY + POINT_SIZE/2});
    };

    int slam_pointX = static_cast<int>(slam_virtual_pos_X);
    int slam_pointY = static_cast<int>(slam_virtual_pos_Y);
    if (!slam_trail.empty()) {
        auto& last = slam_trail.back();
        if (last.first != slam_pointX || last.second != slam_pointY) {
            slam_trail.push_back({slam_pointX + POINT_SIZE/2, slam_pointY + POINT_SIZE/2});
        }
    } else {
        slam_trail.push_back({slam_pointX + POINT_SIZE/2, slam_pointY + POINT_SIZE/2});
    };
    noise_rotation+=slam_obj->noisy_control[1];
    slam_obj->normalizeAngle(noise_rotation);
    
    noisePointX += cos(noise_rotation)*slam_obj->noisy_control[0];
    noisePointY += sin(noise_rotation)*slam_obj->noisy_control[0];
    double logger_array_red[3] = {noisePointX,noisePointY,noise_rotation};
    logger->writeMatrixCoords(logger_array_red,"redVector.txt");
    int noisePointX_int = static_cast<int>(noisePointX);
    int noisePointY_int = static_cast<int>(noisePointY);
    if (!noise_trail.empty()) {
        auto& last = noise_trail.back();
        if (last.first != noisePointX_int || last.second != noisePointY_int) {
            noise_trail.push_back({noisePointX_int + POINT_SIZE/2, noisePointY_int + POINT_SIZE/2});
        }
    } else {
        noise_trail.push_back({noisePointX_int + POINT_SIZE/2, noisePointY_int + POINT_SIZE/2});
    };
};

void VisualApp::OnRender() {
    // Очистка экрана
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // Отрисовка трека
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    for (size_t i = 1; i < trail.size(); i++) {
        SDL_RenderDrawLine(renderer,
                         trail[i-1].first, trail[i-1].second,
                         trail[i].first, trail[i].second);
    }

    
    for (Pose* pose : history_poses_struct){
        SDL_Point points_tringle[4];
        int graph_point_X = static_cast<int>(pose->x);
        int graph_point_Y = static_cast<int>(pose->y);
        // std::cout<< graph_point_X << std::endl;
        // std::cout<< history_poses_struct.size() << std::endl;
        // std::cout<< graph_point_Y << std::endl;
        points_tringle[0] = {graph_point_X,graph_point_Y};
        points_tringle[1] = {graph_point_X+POINT_SIZE,graph_point_Y+POINT_SIZE};
        points_tringle[2] = {graph_point_X-POINT_SIZE,graph_point_Y+POINT_SIZE};
        points_tringle[3] = {graph_point_X,graph_point_Y};
        SDL_RenderDrawLines(renderer,points_tringle,4);
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
    for (auto& landmark: landmarks){
        SDL_Point points_l[5];
        points_l[0] = {landmark.first, landmark.second};
        points_l[1] = {landmark.first + POINT_SIZE,landmark.second};
        points_l[2] = {landmark.first  + POINT_SIZE, landmark.second  + POINT_SIZE};
        points_l[3] = {landmark.first, landmark.second + POINT_SIZE};
        points_l[4] = {landmark.first, landmark.second}; // Замыкаем контур
        // std::cout << "=================" << std::endl;
        // std::cout << "LANDMARK GREEN" << std::endl;
        // std::cout << landmark.first << std::endl;
        // std::cout << landmark.second << std::endl;
        // std::cout << "=================" << std::endl;
        SDL_RenderDrawLines(renderer,points_l,5);
        
    }
    
    SDL_SetRenderDrawColor(renderer, 255, 225, 0, 255);
    SDL_Point points_l_slam1[5];
    SDL_Point points_l_slam2[5];

    points_l_slam1[0] = {landmarks_slam[0].first, landmarks_slam[0].second};
    points_l_slam1[1] = {landmarks_slam[0].first + POINT_SIZE,landmarks_slam[0].second};
    points_l_slam1[2] = {landmarks_slam[0].first + POINT_SIZE, landmarks_slam[0].second + POINT_SIZE};
    points_l_slam1[3] = {landmarks_slam[0].first, landmarks_slam[0].second + POINT_SIZE};
    points_l_slam1[4] = {landmarks_slam[0].first, landmarks_slam[0].second}; // Замыкаем контур

    points_l_slam2[0] = {landmarks_slam[1].first, landmarks_slam[1].second};
    points_l_slam2[1] = {landmarks_slam[1].first + POINT_SIZE,landmarks_slam[1].second};
    points_l_slam2[2] = {landmarks_slam[1].first + POINT_SIZE, landmarks_slam[1].second + POINT_SIZE};
    points_l_slam2[3] = {landmarks_slam[1].first, landmarks_slam[1].second + POINT_SIZE};
    points_l_slam2[4] = {landmarks_slam[1].first, landmarks_slam[1].second}; 


   

    SDL_RenderDrawLines(renderer,points_l_slam1,5);
    
    // 1. Рисуем заполненный прямоугольник
    
    // 2. Рисуем повернутые границы
    //SDL_RenderDrawRectEx(renderer, &rect, rotation, &center);
    
    for (size_t i = 1; i < slam_trail.size(); i++) {
        SDL_RenderDrawLine(renderer,
            slam_trail[i-1].first, slam_trail[i-1].second,
            slam_trail[i].first, slam_trail[i].second);
    }


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
        points2[i] = VisualApp::rotate_point(points2[i], center2, rotation);
    };
    SDL_RenderDrawLines(renderer,points2,5);
    SDL_SetRenderDrawColor(renderer, 225, 0, 0, 255);
    int noisePointX_int = static_cast<int>(noisePointX);
    int noisePointY_int = static_cast<int>(noisePointY);
    
    SDL_Point points_noise[5];
    points_noise[0] = {noisePointX_int, noisePointY_int};
    points_noise[1] = {noisePointX_int + POINT_SIZE,noisePointY_int};
    points_noise[2] = {noisePointX_int + POINT_SIZE, noisePointY_int + POINT_SIZE};
    points_noise[3] = {noisePointX_int, noisePointY_int + POINT_SIZE};
    points_noise[4] = {noisePointX_int, noisePointY_int}; // Замыкаем контур
    SDL_RenderDrawLines(renderer,points_noise,5);

    for (size_t i = 1; i < noise_trail.size(); i++) {
        SDL_RenderDrawLine(renderer,
            noise_trail[i-1].first, noise_trail[i-1].second,
            noise_trail[i].first, noise_trail[i].second);
    }

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