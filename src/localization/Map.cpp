#include <iostream>

#include "Map.hpp"

// basic constructor
Map::Map() : grid(), map_received(false) {}

// receives a OccupancyGrid msg and converts to internal representation
bool Map::updateMap(const nav_msgs::OccupancyGrid &map_msg) {

    // update status
    bool update_status = false;

    map_mutex.lock();

    if (!map_received) {

        // lock the mutex

        // update the grid map
        grid.updateGridMap(map_msg);

        // updates the likelihood
        grid.nearestNeighbor();

        // avoiding unnecessary copies
        update_status = map_received = true;

    }

    // unlock the mutex
    map_mutex.unlock();

    return update_status;
}

// returns the grid map
void Map::getGridMap(GridMap *g) {


    // copy, is it really necessary?
    // lock the mutex
    map_mutex.lock();

    // copy the entire grid map
    g->copy(grid);

    // lock the mutex
    map_mutex.unlock();

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

    double size = grid.width*grid.height;

    // shortcut
    MapCell *cells = grid.cells;

    // get all available cells
    for (int i = 0; i < size; i++) {
        if (-1 == cells[i].occ_state) {
            available.push_back(i);
        }
    }

    return available;

}
// updateMaxOccDist
void Map::updateMaxOccDist(double max) {

    // lock the map
    map_mutex.lock();

    // update the distance
    grid.max_occ_dist = max;

    // unlock the map
    map_mutex.unlock();
}

// spreads the particles over the entire map, randomly
void Map::uniformSpread(SampleSet *Xt) {

    // lock the map
    map_mutex.lock();

    // verify if there's a map and the particles aren't already spreaded
    if (map_received && !Xt->spreaded) {

        // get the available cells indexes
        std::vector<int> indexes = getAvailableCellsIndexes();

        // set a generator engine
        std::default_random_engine generator(std::random_device {} ());

        // set a uniform distribution
        std::uniform_int_distribution<int> uniform_dist(0, indexes.size() - 1);

        // set another uniform distribution -  0 ~ 2PI
        std::uniform_real_distribution<double> angle_dist(0.0, std::atan(1.0)*8);

        // set a normal distribution
        std::normal_distribution<double> normal_dist(0.0, 0.05);

        // tmp variables
        int index;
        double *pose;
        int g_i, g_j;
        float div = 1.0/(float)grid.width;

        // shortcut
        Sample2D *samples = Xt->samples;

        // sledgehammer programing style? =-/
        // create random poses based on the unnoccupied cells
        for (int i = 0; i < Xt->size; i++) {

            // a simple pointer to avoid repetitive writing, another shortcut
            pose = samples[i].pose.v;

            // get a random index from the uniform_int_distribution
            index = uniform_dist(generator);

            // get the grid x coord
            g_i = indexes[index]*div;

            // get the grid y coord
            g_j = indexes[index] % grid.width;

            // now we have the grid index and a we can 
            // convert to world coords and assign to the pose value
            // with a simple gaussian noise
            pose[0] = (grid.origin_x + (g_i - grid.width/2)*grid.scale) + normal_dist(generator);
            pose[1] = (grid.origin_y + (g_j - grid.height/2)*grid.scale) + normal_dist(generator);

            // random orientation between 0 and 2*PI radians
            pose[2] = angle_dist(generator);

        }

    }

    std::cout << "Spreaded!" << std::endl;

    // unlock the map
    map_mutex.unlock();
}
