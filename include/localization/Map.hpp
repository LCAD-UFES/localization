#ifndef MAP_H
#define MAP_H

#include <mutex>

#include "nav_msgs/OccupancyGrid.h"

class Map {
    private:
        // the occupancy grid
        nav_msgs::OccupancyGrid grid;
        // mutex to lock map
        std::mutex map_mutex;
        // 
        bool map_received;

    public:
        Map();
        // updates the grid
        void setGrid(nav_msgs::OccupancyGrid);
        // returns the grid
        nav_msgs::OccupancyGrid getGrid();
        // 

};

#endif