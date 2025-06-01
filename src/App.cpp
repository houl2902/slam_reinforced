
#include "App.hpp"


App::App() {
    running = true;
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
    landmarks;
    history_poses_struct;
    logger = NULL;
    input_file.open("input_commands.txt");

}



bool App::OnInit() {
    if (!input_file.is_open()) {
        std::cerr << "Error opening input file" << std::endl;
        return -1;
    }
    logger = new Logger({"greenVector.txt","yellowVector.txt","redVector.txt","Time.txt"});
    return true;
}

int App::OnExecute(EKFslam* slam_obj, GraphSLAM* graph_slam_obj) {
    if(OnInit() == false) {
        return -1;
    }

    pointX = WINDOW_WIDTH / 2 - POINT_SIZE / 2;
    pointY = WINDOW_HEIGHT / 2 - POINT_SIZE / 2;
    landmarks.push_back({300,300});
    landmarks.push_back({350,400});
    landmarks.push_back({200.0,300.0});
    landmarks.push_back({350.0,450.0});
    landmarks.push_back({500.0,300.0});
    landmarks.push_back({500.0,300.0});
    landmarks.push_back({350.0,600.0});
    landmarks.push_back({100.0,300.0});
    landmarks.push_back({350.0,200.0});
    landmarks.push_back({100.0,500.0});
    landmarks.push_back({380.0,410.0});
    slam_obj->addLandmark(300.0,300.0);
    slam_obj->addLandmark(350.0,400.0);
    slam_obj->addLandmark(200.0,300.0);
    slam_obj->addLandmark(350.0,450.0);
    slam_obj->addLandmark(500.0,300.0);
    slam_obj->addLandmark(350.0,600.0);
    slam_obj->addLandmark(100.0,300.0);
    slam_obj->addLandmark(350.0,200.0);
    slam_obj->addLandmark(100.0,500.0);
    slam_obj->addLandmark(380.0,410.0);
    

    // #ifdef SDL_OFF
    // Режим файлового ввода
    
   
    // #endif

    while(running) {

        // Обработка событий SDL
        // #else
        //Чтение команд из файла
        std::string command;
        if (!(input_file >> command)) {
            running = false; // Конец файла
            continue;
        }
        // #endif

        OnLoop(slam_obj, graph_slam_obj);
        
        
        
    }
    OnCleanup();
    return 0;
}
void App::OnLoop(EKFslam* slam_obj, GraphSLAM* graph_slam_obj){
    static std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    static double total_elapsed_seconds = 0.0;
    double x_move_offset = cos(rotation * M_PI / 180.0)*MOVE_SPEED;
    double y_move_offset = sin(rotation * M_PI / 180.0)*MOVE_SPEED;

    
    double control[2] = {0,0};
    static std::string command;
    static double value;
    if (input_file >> command) {
        if (command == "forward") {
            virtual_pos_X-=x_move_offset;
            virtual_pos_Y-=y_move_offset;
            control[0] = -MOVE_SPEED;
        };
        if (command == "backward") {
            virtual_pos_X+=x_move_offset;
            virtual_pos_Y+=y_move_offset;
            control[0] = MOVE_SPEED;
        };
        if (command == "left") {
            rotation += rotationSpeed;
            control[1] = rotationSpeed*M_PI / 180.0;
        };
        if (command == "right") {
            rotation -= rotationSpeed;
            control[1] = -rotationSpeed*M_PI / 180.0;
        };
    }
    rotation = std::fmod(rotation, 360.0);
    if (rotation > 180.0) rotation -= 360.0;
    if (rotation < -180.0) rotation += 360.0;
    auto loop_start = std::chrono::high_resolution_clock::now();

   

    double logger_array_green[3] = {virtual_pos_X,virtual_pos_Y,rotation};
    logger->writeMatrixCoords(logger_array_green,"greenVector.txt");
    double distance = sqrt((300-virtual_pos_X) * (300-virtual_pos_X) + (300-virtual_pos_Y) * (300-virtual_pos_Y));
    if (distance < 1e-10) distance = 1e-10;
    double angle = atan2(300-virtual_pos_Y,300-virtual_pos_X) - rotation*M_PI / 180.0;
    //double angle = std::atan2(virtual_pos_Y, virtual_pos_X)

    slam_obj->predict(control);
    
    for (int i = 0; i<landmarks.size(); i++){
      double distance = sqrt((landmarks[i].first-virtual_pos_X) * (landmarks[i].first-virtual_pos_X) + (landmarks[i].second-virtual_pos_Y) * (landmarks[i].second-virtual_pos_Y));
      if (distance < 1e-10) distance = 1e-10;
      double angle = atan2(landmarks[i].second-virtual_pos_Y,landmarks[i].first-virtual_pos_X) - rotation*M_PI / 180.0;
      double measurements[2] = {distance,angle};
      if (!std::isnan(measurements[0]) && !std::isnan(measurements[1])){
        slam_obj->update(measurements,i);
      }
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

    if (graph_slam_obj->odometry_constraints.size() > 3) { // Изменено условие
        graph_slam_obj->optimizeGraph(15);
    }
    
    noise_rotation+=slam_obj->noisy_control[1];
    slam_obj->normalizeAngle(noise_rotation);
    auto loop_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> loop_elapsed = loop_end - loop_start;
    double loop_seconds = loop_elapsed.count();
    
    // Суммируем общее время
    total_elapsed_seconds += loop_seconds;
    
    noisePointX += cos(noise_rotation)*slam_obj->noisy_control[0];
    noisePointY += sin(noise_rotation)*slam_obj->noisy_control[0];
    double logger_array_red[3] = {noisePointX,noisePointY,noise_rotation};
    logger->writeMatrixCoords(logger_array_red,"redVector.txt");
    logger->writeFloat(loop_seconds,"Time.txt");
};

void App::OnCleanup() {
    if (logger) {
        delete logger;
    };
    input_file.close();
}