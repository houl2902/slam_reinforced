#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "src/SensorData.hpp"  // Include your sensor data handling class
//#include "src/Map.hpp"         // Include your map class
//#include "src/Optimizer.hpp"   // Include your optimization class

// class SLAM {
// public:
//     SLAM() : Map(), Optimizer() {}

//     void processSensorData(const SensorData& data) {
//         // Step 1: Process the incoming sensor data
//         std::vector<Feature> features = extractFeatures(data);
        
//         // Step 2: Update the map with new features
//         Map.update(features);
        
//         // Step 3: Optimize the map and pose estimates
//         Optimizer.optimize(map);
//     }

    // void run() {
    //     while (true) {
    //         SensorData data = getNextSensorData();
    //         processSensorData(data);
    //         displayMap();
    //     }
    // }

// private:
//     Map Map;
//     Optimizer Optimizer;

//     std::vector<Feature> extractFeatures(const SensorData& data) {

//         // Implement feature extraction logic here
//         std::vector<Feature> features;
//         // Extract features from sensor data
//         return features;
//     }

//     SensorData getNextSensorData() {
//         // Implement logic to get the next sensor data (e.g., from a file or real sensor)
//         SensorData data;
//         return data;
//     }

//     void displayMap() {
//         // Implement logic to visualize or print the map
//         Map.display();
//     }
// };

int main() {
    //SLAM slamSystem;
    //slamSystem.run();
    cv::Mat image;
    image = cv::imread( "", cv::IMREAD_COLOR );
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    return 0;
}