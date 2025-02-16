#include <opencv2/opencv.hpp> 
#include <vector>
#include <algorithm>
#include <list>
#include "featureDetector.hpp"

// Конструктор
featureDetector::featureDetector(int numFeatures, float scaleFactor, int nLevels, int edgeThreshold) {
    orb = cv::ORB::create(numFeatures, scaleFactor, nLevels, edgeThreshold);
}

// Метод для одновременного обнаружения и вычисления дескрипторов
void featureDetector::detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
}
void featureDetector::readImage(cv::Mat image) {
    images.push_back(image);
}

// Метод для отрисовки ключевых точек
void featureDetector::drawKeypoints(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints, cv::Mat& outputImage) {
    cv::drawKeypoints(image, keypoints, outputImage, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}