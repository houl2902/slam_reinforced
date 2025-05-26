// ------------ Процесс 1: EKF SLAM (ekf_process.cpp) ------------
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include "VisualApp.hpp"

struct SharedData {
    double state[10];  // x, y, theta, lx1, ly1, lx2, ly2
    double control[2];
    bool new_data;
    pthread_mutex_t mutex;
};

int main() {
    // Создаем shared memory
    int fd = shm_open("/slam_shared", O_CREAT | O_RDWR, 0666);
    ftruncate(fd, sizeof(SharedData));
    SharedData* shared = (SharedData*)mmap(NULL, sizeof(SharedData), 
                                      PROT_READ | PROT_WRITE, 
                                      MAP_SHARED, fd, 0);
    
    // Инициализация мьютекса
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shared->mutex, &attr);

    EKFslam ekf;
    ekf.addLandmark(300.0, 300.0);
    ekf.addLandmark(350.0, 400.0);

    while (true) {
        // Получаем управление
        pthread_mutex_lock(&shared->mutex);
        double control[2] = {shared->control[0], shared->control[1]};
        pthread_mutex_unlock(&shared->mutex);

        // Выполняем EKF
        ekf.predict(control);
        
        // Обновляем shared memory
        pthread_mutex_lock(&shared->mutex);
        memcpy(shared->state, ekf.state, sizeof(ekf.state));
        shared->new_data = true;
        pthread_mutex_unlock(&shared->mutex);
        
        usleep(10000); // 10 мс
    }
}

// ------------ Процесс 2: GraphSLAM (graph_process.cpp) ------------
#include "GraphSLAM.hpp"

int main() {
    // Подключаем shared memory
    int fd = shm_open("/slam_shared", O_RDWR, 0666);
    SharedData* shared = (SharedData*)mmap(NULL, sizeof(SharedData),
                                      PROT_READ | PROT_WRITE,
                                      MAP_SHARED, fd, 0);

    GraphSLAM graph;
    while (true) {
        pthread_mutex_lock(&shared->mutex);
        if (shared->new_data) {
            graph.addPose(shared->state);
            shared->new_data = false;
            
            auto pose_detected = graph.detectLoop(shared->state);
            if (pose_detected != nullptr) {
                // Обработка петли
            }
        }
        pthread_mutex_unlock(&shared->mutex);
        
        usleep(50000); // 50 мс
    }
}

// ------------ Главный процесс: Визуализация (main.cpp) ------------
int main() {
    // Запуск дочерних процессов
    pid_t ekf_pid = fork();
    if (ekf_pid == 0) {
        execl("./ekf_process", "./ekf_process", NULL);
        exit(1);
    }
    
    pid_t graph_pid = fork();
    if (graph_pid == 0) {
        execl("./graph_process", "./graph_process", NULL);
        exit(1);
    }

    // Инициализация SDL
    VisualApp app;
    if (!app.OnInit()) {
        return -1;
    }

    // Подключение shared memory
    int fd = shm_open("/slam_shared", O_RDWR, 0666);
    SharedData* shared = (SharedData*)mmap(NULL, sizeof(SharedData),
                                      PROT_READ | PROT_WRITE,
                                      MAP_SHARED, fd, 0);

    // Основной цикл визуализации
    SDL_Event event;
    while (app.running) {
        while (SDL_PollEvent(&event)) {
            app.OnEvent(&event);
        }

        // Обработка ввода
        const Uint8* keys = SDL_GetKeyboardState(NULL);
        pthread_mutex_lock(&shared->mutex);
        if (keys[SDL_SCANCODE_W]) shared->control[0] = -app.MOVE_SPEED;
        if (keys[SDL_SCANCODE_S]) shared->control[0] = app.MOVE_SPEED;
        if (keys[SDL_SCANCODE_A]) shared->control[1] = app.rotationSpeed * M_PI / 180.0;
        if (keys[SDL_SCANCODE_D]) shared->control[1] = -app.rotationSpeed * M_PI / 180.0;
        pthread_mutex_unlock(&shared->mutex);

        // Обновление визуализации
        pthread_mutex_lock(&shared->mutex);
        app.slam_virtual_pos_X = shared->state[0];
        app.slam_virtual_pos_Y = shared->state[1];
        app.slam_rotation = shared->state[2];
        pthread_mutex_unlock(&shared->mutex);

        app.OnRender();
        SDL_Delay(16); // ~60 FPS
    }

    // Очистка
    munmap(shared, sizeof(SharedData));
    shm_unlink("/slam_shared");
    kill(ekf_pid, SIGTERM);
    kill(graph_pid, SIGTERM);
    waitpid(ekf_pid, NULL, 0);
    waitpid(graph_pid, NULL, 0);
    
    app.OnCleanup();
    return 0;
}