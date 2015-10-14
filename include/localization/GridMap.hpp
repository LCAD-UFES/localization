#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <queue>

#include <nav_msgs/OccupancyGrid.h>

#include "MapCell.hpp"
#include "CellData.hpp"

#define MAP_INDEX(i, j) ((i) + (j) *width )

class GridMap {

    public:
        // Map origin; the map is a viewport onto a conceptual larger map.
        float origin_x, origin_y;

        // Map scale (m/px)
        float scale;

        // Map dimensions (number of cells)
        int width, height, size;

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

        // 
        void enqueue(
            unsigned int,
            unsigned int,
            unsigned int,
            unsigned int,
            std::priority_queue<CellData> &,
            double**,
            int,
            unsigned char*
        );

        // pre-computing the max occlusion disance
        void nearestNeighbor();

        // copy another GridMap
        void copy(const GridMap&);

        // get the pre-computed likelihood
        double getMinDistance(int i, int j);

};

#endif