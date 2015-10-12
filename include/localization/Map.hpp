#ifndef MAP_H
#define MAP_H

#include <mutex>

#include "nav_msgs/OccupancyGrid.h"

#include "MapCell.hpp"

class Map {
    private:
        // Map origin; the map is a viewport onto a conceptual larger map.
        double origin_x, origin_y;

        // Map scale (m/cell)
        double scale;

        // Map dimensions (number of cells)
        int width, height;

        // The map data, stored as a grid
        MapCell *cells;

        // Max distance at which we care about obstacles, for constructing
        // likelihood field
        double max_occ_dist;

        // mutex to lock map
        std::mutex map_mutex;
        // flag to avoiding unnecessary copies
        bool map_received;

        // deprecated
        nav_msgs::OccupancyGrid grid;

    public:
        Map();
        // updates the grid
        void updateMap(const nav_msgs::OccupancyGrid&);

        // returns the grid
        nav_msgs::OccupancyGrid getGrid();

        // returns the map flag
        bool mapReceived();

        // force map update
        void forceUpdate();

        // get the cells pointer
        std::vector<int> getAvailableCellsIndex();

        // get the map width
        double geWidth();

        // get the map height
        double getHeight();

        // get the map scale
        double getScale();

        // get origin_x
        double getOriginX();

        // get origin_y
        double getOriginY();

};

#endif