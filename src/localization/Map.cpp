#include <iostream>

#include "Map.hpp"

// basic constructor
Map::Map() : cells(nullptr), map_received(false) {}

// receives a OccupancyGrid msg and converts to internal representation
bool Map::updateMap(const nav_msgs::OccupancyGrid &map_msg) {

    // update status
    bool update_status = false;

    if (!map_received) {

        // lock the mutex
        map_mutex.lock();

        // update the map infos
        width = map_msg.info.width;
        height = map_msg.info.height;
        scale  =  map_msg.info.resolution;
        origin_x = map_msg.info.origin.position.x + (width/2)*scale;
        origin_y = map_msg.info.origin.position.y + (height/2)*scale;

        // get the grid in a single row
        // avoiding memmmry leak
        if (nullptr != cells) {
            delete cells;
        }

        // realocate the cells
        cells = new MapCell[width*height]();
        if (nullptr == cells) {
            throw std::bad_alloc();
        }

        // copy the occupancy state
        for (int i = 0; i < width*height; i++) {
            if (0 == map_msg.data[i]) {
                cells[i].occ_state = -1;
            } else if (100 == map_msg.data[i]) {
                cells[i].occ_state = +1;
            } else {
                cells[i].occ_state = 0;
            }
        }

        // updates the likelihood
        nearestNeighbor();

        // avoiding unnecessary copies
        update_status = map_received = true;

        // unlock the mutex
        map_mutex.unlock();
    }

    return update_status;
}

// pre-computing the nearest neighbor
void Map::nearestNeighbor() {
    
}

// returns the grid map
MapCell* Map::getGrid() {

    // copy, is it really necessary?
    // lock the mutex
    map_mutex.lock();

    MapCell *g = new MapCell[width*height]();
    if (nullptr == g) {
        throw std::bad_alloc();
    }

    // copy all values
    for (int i = 0; i < width*height; i++) {
        g[i] = cells[i];
    }

    // lock the mutex
    map_mutex.unlock();

    return g;

}

// force map update
void Map::forceUpdate() {

    // lock the map
    map_mutex.lock();

    // set to false
    map_received = false;

    // unlock the map
    map_mutex.unlock();
}

// returns the map flag
bool Map::mapReceived() {

    // lock the map
    map_mutex.lock();

    // copy the map_received flag
    bool flag = map_received;

    // unlock the map
    map_mutex.unlock();

    return flag;
}

// get the cells pointer
std::vector<int> Map::getAvailableCellsIndexes() {

    std::vector<int> available;

    // lock the map
    map_mutex.lock();

    // get all available cells
    for (int i = 0; i < width*height; i++) {
        if (-1 == cells[i].occ_state) {
            available.push_back(i);
        }
    }

    // unlock the map
    map_mutex.unlock();

    return available;

}

// get the map width
double Map::geWidth() {

    // lock the map
    map_mutex.lock();

    // copy the map_received flag
    double m_width = width;

    // unlock the map
    map_mutex.unlock();

    return m_width;
}

// get the map height
double Map::getHeight() {
    
    // lock the map
    map_mutex.lock();

    // copy the map_received flag
    double m_height = height;

    // unlock the map
    map_mutex.unlock();

    return m_height;
}

// get the map scale
double Map::getScale() {
    
    // lock the map
    map_mutex.lock();

    // copy the map_received flag
    double m_scale = scale;

    // unlock the map
    map_mutex.unlock();

    return m_scale;
}


// get origin_x
double Map::getOriginX() {

    // lock the map
    map_mutex.lock();

    // copy the map_received flag
    double o_x = origin_x;

    // unlock the map
    map_mutex.unlock();

    return o_x;

}

// get origin_y
double Map::getOriginY() {

    // lock the map
    map_mutex.lock();

    // copy the map_received flag
    double o_y = origin_y;

    // unlock the map
    map_mutex.unlock();

    return o_y;

}
