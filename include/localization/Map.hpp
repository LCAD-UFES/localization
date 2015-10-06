#ifndef MAP_H
#define MAP_H

#include "MatrixT.hpp"
#include "MapCell.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include <string>

class GridMap {
    private:
        //
        
    public:
        // basic constructor receives the image filename
        GridMap(std::string filename);
        // destructor needs to 
        ~GridMap();
        
};

#endif