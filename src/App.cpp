
#include "App.hpp"

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <iostream>

#include <cstdlib>  // Для rand() и srand()
#include <ctime>    // Для time()

class Map {
    private:
        int width;
        int height;
        std::vector<std::vector<bool>> grid; // true means obstacle
    
    public:
        Map(int w, int h) : width(w), height(h), grid(h, std::vector<bool>(w, false)) {}
    
        void setObstacle(int x, int y, bool isObstacle) {
            if (x < 0 && x >= width && y < 0 || y >= height) {
                throw std::out_of_range("Map coordinates out of range");
            }
            grid[y][x] = isObstacle;
        }
    
        bool isObstacle(int x, int y) const {
            if (x < 0 && x >= width && y < 0 || y >= height) {
                return true; // Consider out of bounds as obstacle
            }
            return grid[y][x];
        }
    
        int getWidth() const { return width; }
        int getHeight() const { return height; }
    
        void printMap() const {
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    std::cout << (grid[y][x] ? 'X' : '.');
                }
                std::cout << std::endl;
            }
        }
    };
    
    class AStar {
        private:
            struct Node {
                int x, y;
                double g, h, f;
                Node* parent;
        
                Node(int x, int y, double g, double h, Node* parent = nullptr)
                    : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}
        
                bool operator>(const Node& other) const {
                    return f > other.f;
                }
            };
        
            Map map;
            double heuristic(int x1, int y1, int x2, int y2) {
                return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
            }
        
        public:
            AStar(int width, int height) : map(width, height) {}
        
            void setObstacle(int x, int y, bool isObstacle) {
                map.setObstacle(x, y, isObstacle);
            }
        
            std::vector<std::pair<int, int>> findPath(int startX, int startY, int goalX, int goalY) {
                if (map.isObstacle(startX, startY) || map.isObstacle(goalX, goalY)) {
                    return {}; // No path if start or goal is obstacle
                }
        
                std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
                std::vector<std::vector<bool>> closedSet(map.getHeight(), std::vector<bool>(map.getWidth(), false));
                std::vector<std::vector<Node*>> nodes(map.getHeight(), std::vector<Node*>(map.getWidth(), nullptr));
        
                Node* startNode = new Node(startX, startY, 0, heuristic(startX, startY, goalX, goalY));
                openSet.push(*startNode);
                nodes[startY][startX] = startNode;
        
                const int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
                const int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};
        
                while (!openSet.empty()) {
                    Node current = openSet.top();
                    openSet.pop();
        
                    if (current.x == goalX && current.y == goalY) {
                        // Reconstruct path
                        std::vector<std::pair<int, int>> path;
                        Node* node = nodes[current.y][current.x];
                        while (node != nullptr) {
                            path.emplace_back(node->x, node->y);
                            node = node->parent;
                        }
                        std::reverse(path.begin(), path.end());
        
                        // Clean up
                        for (int y = 0; y < map.getHeight(); ++y) {
                            for (int x = 0; x < map.getWidth(); ++x) {
                                delete nodes[y][x];
                            }
                        }
                        return path;
                    }
        
                    closedSet[current.y][current.x] = true;
        
                    for (int i = 0; i < 8; ++i) {
                        int nx = current.x + dx[i];
                        int ny = current.y + dy[i];
        
                        if (nx < 0  && nx >= map.getWidth() && ny < 0  && ny >= map.getHeight()  &&
                            map.isObstacle(nx, ny) || closedSet[ny][nx]) {
                            continue;
                        }
        
                        double newG = current.g + ((i < 4) ? 1.0 : 1.414); // sqrt(2) for diagonal
                        double newH = heuristic(nx, ny, goalX, goalY);
                        double newF = newG + newH;
        
                        if (nodes[ny][nx] == nullptr || newF < nodes[ny][nx]->f) {
                            if (nodes[ny][nx] != nullptr) {
                                // Need to update priority in openSet, but priority_queue doesn't support it
                                // So we just add a new node (the old one will be ignored when we find it in closedSet)
                            }
                            Node* newNode = new Node(nx, ny, newG, newH, nodes[current.y][current.x]);
                            openSet.push(*newNode);
                            nodes[ny][nx] = newNode;
                        }
                    }
                }
        
                // Clean up
                for (int y = 0; y < map.getHeight(); ++y) {
                    for (int x = 0; x < map.getWidth(); ++x) {
                        delete nodes[y][x];
                    }
                }
        
                return {}; // No path found
            }
        
            void printMapWithPath(int startX, int startY, int goalX, int goalY) {
                auto path = findPath(startX, startY, goalX, goalY);
                std::ofstream  Map;
                Map.open("Map.txt");
                for (int y = 0; y < map.getHeight(); ++y) {
                    for (int x = 0; x < map.getWidth(); ++x) {
                        if (x == startX && y == startY) {
                            Map << 'S';
                        } else if (x == goalX && y == goalY) {
                            Map << 'G';
                        } else if (map.isObstacle(x, y)) {
                            Map << 'X';
                        } else if (std::find(path.begin(), path.end(), std::make_pair(x, y)) != path.end()) {
                            Map << '*';
                        } else {
                            Map << '.';
                        }
                    }
                    Map << std::endl;
                }
                Map.close();
            }

            std::vector<std::string> convertPathToCommands(const std::vector<std::pair<int, int>>& path) {
                std::vector<std::string> commands;
                if (path.size() < 2) return commands;
            
                // Начальное направление - вдоль оси X (вправо)
                int currentDirX = 1;
                int currentDirY = 0;
            
                for (size_t i = 1; i < path.size(); ++i) {
                    int dx = path[i].first - path[i-1].first;
                    int dy = path[i].second - path[i-1].second;
            
                    // Определяем новое направление
                    int newDirX = dx;
                    int newDirY = dy;
            
                    // Если направление не изменилось - движение вперед
                    if (newDirX == currentDirX && newDirY == currentDirY) {
                        commands.push_back("forward");
                        continue;
                    }
            
                    // Определяем поворот
                    if (newDirX == -currentDirX && newDirY == -currentDirY) {
                        // Разворот на 180 градусов
                        commands.push_back("right");
                        commands.push_back("right");
                    } else {
                        // Векторное произведение для определения направления поворота
                        int cross = currentDirX * newDirY - currentDirY * newDirX;
                        
                        if (cross > 0) {
                            // Поворот налево (90 градусов против часовой стрелки)
                            commands.push_back("left");
                        } else {
                            // Поворот направо (90 градусов по часовой стрелке)
                            commands.push_back("right");
                        }
            
                        // Проверяем, нужно ли еще поворачивать (для диагональных движений)
                        int dot = currentDirX * newDirX + currentDirY * newDirY;
                        if (dot == 0) {
                            // После первого поворота направление перпендикулярно - нужен еще один поворот
                            commands.push_back("left"); // или right, зависит от системы координат
                        }
                    }
            
                    // Добавляем команду движения вперед
                    commands.push_back("forward");
            
                    // Обновляем текущее направление
                    currentDirX = newDirX;
                    currentDirY = newDirY;
                }
            
                return commands;
            }
        };


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
    logger = new Logger({"greenVector.txt","yellowVector.txt","redVector.txt","Time.txt","Map.txt"});
    return true;
}

int App::OnExecute(EKFslam* slam_obj, GraphSLAM* graph_slam_obj) {
    static std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    if(OnInit() == false) {
        return -1;
    }

    pointX = WINDOW_WIDTH / 2 - POINT_SIZE / 2;
    pointY = WINDOW_HEIGHT / 2 - POINT_SIZE / 2;
    // landmarks.push_back({300,300});
    // landmarks.push_back({350,400});
    // landmarks.push_back({200.0,300.0});
    // landmarks.push_back({350.0,326.0});
    // landmarks.push_back({400.0,300.0});
    // landmarks.push_back({100.0,300.0});
    // landmarks.push_back({350.0,200.0});
    // landmarks.push_back({353.0,304.0});
    // landmarks.push_back({353.0,304.0});
    // landmarks.push_back({100.0,300.0});
    // landmarks.push_back({350.0,200.0});
    // landmarks.push_back({100.0,400.0});
    // landmarks.push_back({380.0,236.0});
    // landmarks.push_back({385.0,376.0});
    std::srand(std::time(0)); // Инициализация генератора случайных чисел
        
    for (int y = 0; y < 50; ++y) {
        for (int x = 0; x < 50; ++x) {
            // Генерируем случайное число от 0.0 до 1.0
            double randomValue = 200*static_cast<double>(std::rand()) / RAND_MAX;
            double randomValue2 = 200*static_cast<double>(std::rand()) / RAND_MAX;
            landmarks.push_back({randomValue,randomValue2});
        }
    }
    slam_obj->addLandmark(300.0,300.0);
    slam_obj->addLandmark(350.0,400.0);
    AStar nav(200,200);
    // slam_obj->addLandmark(200.0,300.0);
    // slam_obj->addLandmark(350.0,450.0);
    // slam_obj->addLandmark(500.0,300.0);
    // slam_obj->addLandmark(350.0,600.0);
    // slam_obj->addLandmark(100.0,300.0);
    // slam_obj->addLandmark(350.0,200.0);
    // slam_obj->addLandmark(100.0,500.0);
    // slam_obj->addLandmark(380.0,410.0);
    for (auto& l : landmarks){
        nav.setObstacle(static_cast<int>(l.first),static_cast<int>(l.second),true);
    }
    nav.printMapWithPath(100,100,146,57);

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
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> loop_elapsed = end_time - start_time ;
    double loop_seconds = loop_elapsed.count();
    logger->writeFloat(loop_seconds,"Time.txt");
    // slam_obj->matrixOps.time_on_matix;
    // graph_slam_obj->matrixOps.time_on_matix;
    std::cout<<graph_slam_obj->matrixOps.counter << std::endl;
    std::cout<<slam_obj->matrixOps.counter << std::endl;
    std::cout<<slam_obj->matrixOps.time_on_matix << std::endl;
    std::cout<<slam_obj->matrixOps.time_on_matix << std::endl;
    logger->writeFloat(slam_obj->matrixOps.time_on_matix,"Time.txt");
    logger->writeFloat(graph_slam_obj->matrixOps.time_on_matix,"Time.txt");
    OnCleanup();
    return 0;
}
void App::OnLoop(EKFslam* slam_obj, GraphSLAM* graph_slam_obj){
    // static std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    // static double total_elapsed_seconds = 0.0;
    // double x_move_offset = cos(rotation * M_PI / 180.0)*MOVE_SPEED;
    // double y_move_offset = sin(rotation * M_PI / 180.0)*MOVE_SPEED;

    
    // double control[2] = {0,0};
    // static std::string command;
    // static double value;
    // if (input_file >> command) {
    //     if (command == "forward") {
    //         virtual_pos_X-=x_move_offset;
    //         virtual_pos_Y-=y_move_offset;
    //         control[0] = -MOVE_SPEED;
    //     };
    //     if (command == "backward") {
    //         virtual_pos_X+=x_move_offset;
    //         virtual_pos_Y+=y_move_offset;
    //         control[0] = MOVE_SPEED;
    //     };
    //     if (command == "left") {
    //         rotation += rotationSpeed;
    //         control[1] = rotationSpeed*M_PI / 180.0;
    //     };
    //     if (command == "right") {
    //         rotation -= rotationSpeed;
    //         control[1] = -rotationSpeed*M_PI / 180.0;
    //     };
    // }
    // rotation = std::fmod(rotation, 360.0);
    // if (rotation > 180.0) rotation -= 360.0;
    // if (rotation < -180.0) rotation += 360.0;
    // auto loop_start = std::chrono::high_resolution_clock::now();

   

    // double logger_array_green[3] = {virtual_pos_X,virtual_pos_Y,rotation};
    // logger->writeMatrixCoords(logger_array_green,"greenVector.txt");
    // double distance = sqrt((300-virtual_pos_X) * (300-virtual_pos_X) + (300-virtual_pos_Y) * (300-virtual_pos_Y));
    // if (distance < 1e-10) distance = 1e-10;
    // double angle = atan2(300-virtual_pos_Y,300-virtual_pos_X) - rotation*M_PI / 180.0;
    // //double angle = std::atan2(virtual_pos_Y, virtual_pos_X)

    // slam_obj->predict(control);
    
    // for (int i = 0; i<landmarks.size(); i++){
    //   double distance = sqrt((landmarks[i].first-virtual_pos_X) * (landmarks[i].first-virtual_pos_X) + (landmarks[i].second-virtual_pos_Y) * (landmarks[i].second-virtual_pos_Y));
    //   if (distance < 1e-10) distance = 1e-10;
    //   double angle = atan2(landmarks[i].second-virtual_pos_Y,landmarks[i].first-virtual_pos_X) - rotation*M_PI / 180.0;
    //   double measurements[2] = {distance,angle};
    //   if (!std::isnan(measurements[0]) && !std::isnan(measurements[1])){
    //     slam_obj->update(measurements,i);
    //   }
    // }
    // //double angle = std::atan2(virtual_pos_Y, virtual_pos_X)
    

    // slam_virtual_pos_X = slam_obj->state[0];
    // slam_virtual_pos_Y = slam_obj->state[1];
    // slam_rotation = slam_obj->state[2];

    // graph_slam_obj->addPose(slam_obj->state, slam_obj->curr_measurement);
    // history_poses_struct = graph_slam_obj->history_poses_struct;

    // logger->writeMatrixCoords(slam_obj->state,"yellowVector.txt");
    
   

    // // Ограничение перемещения точки в пределах окна
    // if (rotation > 360) rotation -= 360;
    // if (rotation < 0) rotation += 360;
    // if (pointX < 0) pointX = 0;
    // if (pointY < 0) pointY = 0;

    // pointX = static_cast<int>(virtual_pos_X);
    // pointY = static_cast<int>(virtual_pos_Y);

    // if (graph_slam_obj->odometry_constraints.size() > 3) { // Изменено условие
    //     graph_slam_obj->optimizeGraph(15);
    // }
    
    // noise_rotation+=slam_obj->noisy_control[1];
    // slam_obj->normalizeAngle(noise_rotation);
    // auto loop_end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> loop_elapsed = loop_end - loop_start;
    // double loop_seconds = loop_elapsed.count();
    
    // // Суммируем общее время
    // total_elapsed_seconds += loop_seconds;
    
    // noisePointX += cos(noise_rotation)*slam_obj->noisy_control[0];
    // noisePointY += sin(noise_rotation)*slam_obj->noisy_control[0];
    // double logger_array_red[3] = {noisePointX,noisePointY,noise_rotation};
    // logger->writeMatrixCoords(logger_array_red,"redVector.txt");
    
};

void App::OnCleanup() {
    if (logger) {
        delete logger;
    };
    input_file.close();
}