#include "GridMap.hpp"

// basic constructor
// max_occ_dist default to 2.0
GridMap::GridMap() : max_occ_dist(2.0) {}

// Copy Constructor
GridMap::GridMap(const GridMap &g) : 
                                    origin_x(g.origin_x),
                                    origin_y(g.origin_y),
                                    scale(g.scale),
                                    width(g.width),
                                    height(g.height),
                                    max_occ_dist(g.max_occ_dist) {

    int size = g.width*g.height;

    // allocate the map cells
    cells = new MapCell[size];
    if (nullptr == cells) {
        throw std::bad_alloc();
    }

    // copy all cells
    for (int i = 0; i < size; i++) {
        cells[i] = g.cells[i];
    }
}

// destructor
GridMap::~GridMap() {
    if (nullptr == cells) {
        delete cells;
    }
}

// update the grid mad
void GridMap::updateGridMap(const nav_msgs::OccupancyGrid &map_msg) {

    // get the size
    int size = map_msg.info.width*map_msg.info.height;

    // maybe we don't need to delete de cells
    if (width != map_msg.info.width || height != map_msg.info.height)  {

        if (nullptr != cells) {
            delete cells;
        }

        // update the map infos
        width = map_msg.info.width;
        height = map_msg.info.height;

        // realocate the cells
        cells = new MapCell[size];
        if (nullptr == cells) {
            throw std::bad_alloc();
        }
    }

    scale  =  map_msg.info.resolution;
    origin_x = map_msg.info.origin.position.x + (map_msg.info.width/2)*map_msg.info.resolution;
    origin_y = map_msg.info.origin.position.y + (map_msg.info.height/2)*map_msg.info.resolution;


    // copy the occupancy state
    for (int i = 0; i < size; i++) {
        if (0 == map_msg.data[i]) {
            cells[i].occ_state = -1;
        } else if (100 == map_msg.data[i]) {
            cells[i].occ_state = +1;
        } else {
            cells[i].occ_state = 0;
        }
    }
}

// update the grid mad, now with another GridMap, overloading
void GridMap::copy(const GridMap &g) {

    // get the size
    int size = g.width*g.height;

    // maybe we don't need to delete de cells
    if (width != g.width || height != g.height)  {

        if (nullptr != cells) {
            delete cells;
        }

        // update the map infos
        width = g.width;
        height = g.height;

        // realocate the cells
        cells = new MapCell[size];
        if (nullptr == cells) {
            throw std::bad_alloc();
        }
    } else if (nullptr == cells) {
        // realocate the cells
        cells = new MapCell[size];
        if (nullptr == cells) {
            throw std::bad_alloc();
        }
    }

    scale  =  g.scale;
    origin_x = g.origin_x;
    origin_y = g.origin_y;


    // copy the occupancy state
    for (int i = 0; i < size; i++) {
        cells[i] = g.cells[i];
    }

    // the max occlusion distance parameter
    max_occ_dist = g.max_occ_dist;

}
