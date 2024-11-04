#include "Feature.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
class SensorData

{
    public:
      cv::Mat image;
      Feature extractFeature(cv::Mat image);   
};