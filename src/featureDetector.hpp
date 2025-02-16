#include <opencv2/opencv.hpp> 
#include <vector>
#include <list>

class featureDetector 
{
    private:
       cv::Ptr<cv::ORB> orb; 
      
    public:
       featureDetector(int numFeatures = 500, float scaleFactor = 1.2f, int nLevels = 8, int edgeThreshold = 31);
       std::list<cv::Mat> images;
       void readImage(cv::Mat image);
       void drawKeypoints(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints, cv::Mat& outputImage);
       void detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

};