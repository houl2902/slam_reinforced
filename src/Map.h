#include "PointLib.h"
#include <unordered_map>
#include <tuple> 
#include <vector>

class Map

{
    private:
      std::unordered_map<std::tuple<int,int,int>,Point> feature_map;
    public:
      Point getPoint(int x, int y, int z);
      void addPoint(Point* point);

}
