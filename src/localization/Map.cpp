#include "Map.hpp"

void Map::setGrid(nav_msgs::OccupancyGrid g) {

    // lock the mutex
    map_mutex.lock();
    grid = g;
    // lock the mutex
    map_mutex.unlock();
}

nav_msgs::OccupancyGrid Map::getGrid() {

    // copy, is it really necessary?
    // lock the mutex
    map_mutex.lock();
    nav_msgs::OccupancyGrid g = grid;
    // lock the mutex
    map_mutex.unlock();

    return g;

}