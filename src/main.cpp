#include <stdlib.h>
#include "Map.h"
#include <iostream>

int main() {
    Map* main_map = new Map;
    Point* new_point= new Point{1,3,4};
    main_map->addPoint(new_point);
    std::cout << "INIT ENDED";
    return 0;
}
