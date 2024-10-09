#include "PointLib.h"
#include "hashFunc.h"
#include <unordered_map>
#include <vector>

class Map

{
    private:
      std::unordered_map<feature,std::vector<Point>> feature_map;
    public:
      Point getPoint(int x, int y, int z);
      void addPoint(Point point);

}
