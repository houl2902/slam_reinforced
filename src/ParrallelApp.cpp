
#include "ParrallelApp.hpp"


ParrallelApp::ParrallelApp() {
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
    control[0] = 0;
    control[1] = 0;
    input_file.open("input_commands.txt");

}



bool ParrallelApp::OnInit() {
    if (!input_file.is_open()) {
        std::cerr << "Error opening input file" << std::endl;
        return -1;
    }
    logger = new Logger({"greenVector.txt","yellowVector.txt","redVector.txt","Time.txt"});
    return true;
}

int ParrallelApp::OnExecute(EKFslam* slam_obj, GraphSLAM* graph_slam_obj) {
    auto program_start = std::chrono::high_resolution_clock::now(); // <- Добавить здесь

    if(OnInit() == false) {
        return -1;
    }

    // pointX = WINDOW_WIDTH / 2 - POINT_SIZE / 2;
    // pointY = WINDOW_HEIGHT / 2 - POINT_SIZE / 2;
    landmarks.push_back({300,300});
    landmarks.push_back({350,400});
    // landmarks.push_back({200.0,300.0});
    // landmarks.push_back({350.0,450.0});
    // landmarks.push_back({500.0,300.0});
    // landmarks.push_back({500.0,300.0});
    // landmarks.push_back({350.0,600.0});
    // landmarks.push_back({100.0,300.0});
    // landmarks.push_back({350.0,200.0});
    // landmarks.push_back({100.0,500.0});
    // landmarks.push_back({380.0,410.0});
    slam_obj->addLandmark(300.0,300.0);
    slam_obj->addLandmark(350.0,400.0);
    // slam_obj->addLandmark(200.0,300.0);
    // slam_obj->addLandmark(350.0,450.0);
    // slam_obj->addLandmark(500.0,300.0);
    // slam_obj->addLandmark(350.0,600.0);
    // slam_obj->addLandmark(100.0,300.0);
    // slam_obj->addLandmark(350.0,200.0);
    // slam_obj->addLandmark(100.0,500.0);
    // slam_obj->addLandmark(380.0,410.0);
    

    // #ifdef SDL_OFF
    // Режим файлового ввода
    
   
    // #endif
    running = true;
    ekf_thread = std::thread(&ParrallelApp::EKFThreadFunc, this, slam_obj);
    graph_thread = std::thread(&ParrallelApp::GraphThreadFunc, this, graph_slam_obj, slam_obj);

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
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        static double total_elapsed_seconds = 0.0;
        double x_move_offset = cos(rotation * M_PI / 180.0)*MOVE_SPEED;
        double y_move_offset = sin(rotation * M_PI / 180.0)*MOVE_SPEED;
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
        double logger_array_green[3] = {virtual_pos_X,virtual_pos_Y,rotation};
        logger->writeMatrixCoords(logger_array_green,"greenVector.txt");
        //OnLoop(slam_obj, graph_slam_obj);
    }; 
    
    }
    auto program_end = std::chrono::high_resolution_clock::now(); 
    std::chrono::duration<double> program_elapsed = program_end - program_start;
    double loop_seconds = program_elapsed.count();
    logger->writeFloat(loop_seconds,"Time.txt");
    if (ekf_thread.joinable()) ekf_thread.join();
    if (graph_thread.joinable()) graph_thread.join();
    OnCleanup();
    return 0;
}
// void ParrallelApp::OnLoop(EKFslam* slam_obj, GraphSLAM* graph_slam_obj){
//     static std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
//     static double total_elapsed_seconds = 0.0;
//     double x_move_offset = cos(rotation * M_PI / 180.0)*MOVE_SPEED;
//     double y_move_offset = sin(rotation * M_PI / 180.0)*MOVE_SPEED;

    
//     double control[2] = {0,0};
//     static std::string command;
//     static double value;
    
//     auto loop_start = std::chrono::high_resolution_clock::now();

   

//     double logger_array_green[3] = {virtual_pos_X,virtual_pos_Y,rotation};
//     logger->writeMatrixCoords(logger_array_green,"greenVector.txt");
//     double distance = sqrt((300-virtual_pos_X) * (300-virtual_pos_X) + (300-virtual_pos_Y) * (300-virtual_pos_Y));
//     if (distance < 1e-10) distance = 1e-10;
//     double angle = atan2(300-virtual_pos_Y,300-virtual_pos_X) - rotation*M_PI / 180.0;
//     //double angle = std::atan2(virtual_pos_Y, virtual_pos_X)

//     slam_obj->predict(control);
    
//     for (int i = 0; i<landmarks.size(); i++){
//       double distance = sqrt((landmarks[i].first-virtual_pos_X) * (landmarks[i].first-virtual_pos_X) + (landmarks[i].second-virtual_pos_Y) * (landmarks[i].second-virtual_pos_Y));
//       if (distance < 1e-10) distance = 1e-10;
//       double angle = atan2(landmarks[i].second-virtual_pos_Y,landmarks[i].first-virtual_pos_X) - rotation*M_PI / 180.0;
//       double measurements[2] = {distance,angle};
//       if (!std::isnan(measurements[0]) && !std::isnan(measurements[1])){
//         slam_obj->update(measurements,i);
//       }
//     }
//     //double angle = std::atan2(virtual_pos_Y, virtual_pos_X)
    

//     slam_virtual_pos_X = slam_obj->state[0];
//     slam_virtual_pos_Y = slam_obj->state[1];
//     slam_rotation = slam_obj->state[2];

//     graph_slam_obj->addPose(slam_obj->state, slam_obj->curr_measurement);
//     history_poses_struct = graph_slam_obj->history_poses_struct;

//     logger->writeMatrixCoords(slam_obj->state,"yellowVector.txt");
    
   

//     // Ограничение перемещения точки в пределах окна
//     if (rotation > 360) rotation -= 360;
//     if (rotation < 0) rotation += 360;
//     if (pointX < 0) pointX = 0;
//     if (pointY < 0) pointY = 0;

//     pointX = static_cast<int>(virtual_pos_X);
//     pointY = static_cast<int>(virtual_pos_Y);

//     if (graph_slam_obj->odometry_constraints.size() > 3) { // Изменено условие
//         graph_slam_obj->optimizeGraph(15);
//     }
    
//     noise_rotation+=slam_obj->noisy_control[1];
//     slam_obj->normalizeAngle(noise_rotation);
//     auto loop_end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> loop_elapsed = loop_end - loop_start;
//     double loop_seconds = loop_elapsed.count();
    
//     // Суммируем общее время
//     total_elapsed_seconds += loop_seconds;
    
    
//     logger->writeFloat(loop_seconds,"Time.txt");
// };

void ParrallelApp::EKFThreadFunc(EKFslam* slam_obj) {
    while(running) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        double local_control[2] = {0, 0};
        double local_virtual_pos_X, local_virtual_pos_Y, local_rotation;
        // Блокируем мьютекс для доступа к общим данным
        
        {
            std::unique_lock<std::mutex> lock(data_mutex);
            local_control[0] = control[0];
            local_control[1] = control[1];
            local_virtual_pos_X = virtual_pos_X;
            local_virtual_pos_Y = virtual_pos_Y;
            local_rotation = rotation;
        }
        
        // Выполняем предсказание EKF
         // Здесь должны быть реальные управления
        slam_obj->predict(local_control);
        
        // Обновление EKF
        for (int i = 0; i < landmarks.size(); i++) {
            double distance = sqrt((landmarks[i].first-virtual_pos_X) * (landmarks[i].first-virtual_pos_X) + 
                            (landmarks[i].second-virtual_pos_Y) * (landmarks[i].second-virtual_pos_Y));
            if (distance < 1e-10) distance = 1e-10;
            double angle = atan2(landmarks[i].second-virtual_pos_Y, landmarks[i].first-virtual_pos_X) - rotation*M_PI/180.0;
            double measurements[2] = {distance, angle};
            
            if (!std::isnan(measurements[0]) && !std::isnan(measurements[1])) {
                slam_obj->update(measurements, i);
            }
        }
        noise_rotation+=slam_obj->noisy_control[1];
        slam_obj->normalizeAngle(noise_rotation);
        noisePointX += cos(noise_rotation)*slam_obj->noisy_control[0];
        noisePointY += sin(noise_rotation)*slam_obj->noisy_control[0];
        double logger_array_red[3] = {noisePointX,noisePointY,noise_rotation};
        logger->writeMatrixCoords(logger_array_red,"redVector.txt");
        logger->writeMatrixCoords(slam_obj->state,"yellowVector.txt");
        // Задержка для контроля частоты обновления
        auto loop_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = loop_end - loop_start;
        std::this_thread::sleep_for(std::chrono::milliseconds(10) - elapsed);
    }
}

void ParrallelApp::GraphThreadFunc(GraphSLAM* graph_slam_obj, EKFslam* slam_obj) {
    while(running) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        double local_state[23];
        double local_measurement[2];
        Pose*  new_pose = nullptr;
        {
            
            std::lock_guard<std::mutex> lock(data_mutex);
            // Копируем состояние поэлементно
            for(int i = 0; i < 23; ++i) {
                local_state[i] = slam_obj->state[i];
            }
            //local_state = slam_obj->state;
            
            // Копируем измерения поэлементно
            for(int i = 0; i < 2; ++i) {
                local_measurement[i] = slam_obj->curr_measurement[i];
            }
          
            //local_measurement = slam_obj->curr_measurement;
            
            Pose* new_pose = graph_slam_obj->addPose(local_state, local_measurement);
        }
      
        // Оптимизация графа (реже, чем обновление)
        Pose* loop_pose_candidate = graph_slam_obj->detectLoop(local_state);
        if (loop_pose_candidate != nullptr && new_pose!=nullptr) {
            std::cout << "Обнаружен цикл! Текущая поза " << new_pose->id << " совпадает с исторической позой " << loop_pose_candidate->id << std::endl;
            graph_slam_obj->addLoopClosureConstraint(new_pose->id, loop_pose_candidate->id);
        }

        if (graph_slam_obj->odometry_constraints.size() > 3) {
          graph_slam_obj->optimizeGraph(15);
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            
            history_poses_struct = graph_slam_obj->history_poses_struct;
         
        }
        
        // Задержка для контроля частоты обновления
        auto loop_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = loop_end - loop_start;
        std::this_thread::sleep_for(std::chrono::milliseconds(20) - elapsed);
    }
}

void ParrallelApp::OnCleanup() {
    running = false; // Сигнал потокам на завершение
    
    if (ekf_thread.joinable()) ekf_thread.join();
    if (graph_thread.joinable()) graph_thread.join();
    if (logger) {
        delete logger;
    };
    input_file.close();
}