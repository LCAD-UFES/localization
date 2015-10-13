#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <nav_msgs/OccupancyGrid.h>

#include "MapCell.hpp"

class GridMap {

    public:
        // Map origin; the map is a viewport onto a conceptual larger map.
        float origin_x, origin_y;

        // Map scale (m/px)
        float scale;

        // Map dimensions (number of cells)
        int width, height;

        // The map data, stored as a grid
        MapCell *cells;

        // Max distance at which we care about obstacles, for constructing
        // likelihood field
        float max_occ_dist;

        // basic constructor
        GridMap ();
        // Copy Constructor
        GridMap (const GridMap&);
        // destructor
        ~GridMap();

        // update the grid map with a new ros OccupancyGrid msg
        void updateGridMap(const nav_msgs::OccupancyGrid&);

        // copy another GridMap
        void copy(const GridMap&);

};

#endif