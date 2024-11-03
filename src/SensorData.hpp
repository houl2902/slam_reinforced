#include "Feature.hpp"

class SensorData

{
    public:
      std::vector<Point> image;
      Feature extractFeature(std::vector<Point> image);   
};