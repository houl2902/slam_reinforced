#include "src/featureDetector.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Загрузка изображения
    cv::Mat image = cv::imread("images/image.jpg", cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Ошибка: не удалось загрузить изображение!" << std::endl;
        return -1;
    }

    // Создание объекта кастомного ORB
    featureDetector customORB;
    customORB.readImage(image);
    // Обнаружение ключевых точек и вычисление дескрипторов
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    customORB.detectAndCompute(image, keypoints, descriptors);

    // Отрисовка ключевых точек
    cv::Mat outputImage;
    customORB.drawKeypoints(image, keypoints, outputImage);

    // Отображение результата
    cv::imshow("Custom ORB Keypoints", outputImage);
    cv::waitKey(0);

    return 0;
}