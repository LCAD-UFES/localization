#ifndef MAP_H
#define MAP_H

#include <mutex>

#include "nav_msgs/OccupancyGrid.h"

#include "MapCell.hpp"

class Map {
    private:
        // Map origin; the map is a viewport onto a conceptual larger map.
        double origin_x, origin_y;

        // Map scale (m/px)
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

    public:
        Map();
        // updates the grid
        bool updateMap(const nav_msgs::OccupancyGrid&);

        // pre-compute the neares neighbor - see LikelihoodFieldModel
        void nearestNeighbor();

        // returns the grid
        MapCell* getGrid();

        // returns the map flag
        bool mapReceived();

        // force map update
        void forceUpdate();

        // get the cells pointer
        std::vector<int> getAvailableCellsIndexes();

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