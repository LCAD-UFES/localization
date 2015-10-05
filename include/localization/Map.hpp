#ifndef MAP_H
#define MAP_H

#include "MapCell.hpp"
#include <string>

class Map {
    private:
        // Map origin; the map is a viewport onto a conceptual larger map.
        double origin_x, origin_y;

        // Map scale (m/cell)
        double scale;

        // Map dimensions (number of cells)
        int size_x, size_y;

        // The map data, stored as a grid
        MapCell *cells;

        // Max distance at which we care about obstacles, for constructing
        // likelihood field
        double max_occ_dist;
    public:
        Map(std::string filename);
        
};

#endif