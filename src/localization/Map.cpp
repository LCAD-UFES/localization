#include <iostream>

#include "Map.hpp"

// basic constructor
Map::Map() : grid(), map_received(false), map_copy(false), normal_dist(0.0, 0.05), generator(std::random_device {} ()) {}

// receives a OccupancyGrid msg and converts to internal representation
bool Map::updateMap(const nav_msgs::OccupancyGrid &map_msg) {

    // update status
    bool update_status = false;

    map_mutex.lock();

    if (!map_received) {

        // lock the mutex

        //Only to Beam Model ray cast
        map = map_msg;

        // update the availableCells
        updateAvailableCells();

        // build the gridmap
        buildGridMap();

        // avoiding unnecessary copies
        update_status = map_received = true;

    }

    // unlock the mutex
    map_mutex.unlock();

    return update_status;

}

void Map::buildGridMap() {

    // lock the mutex
    map_mutex.lock();

    // update the grid map
    grid.updateGridMap(map);

    // updates the likelihood
    grid.nearestNeighbor();

    // unlock the map
    map_mutex.unlock();

}

// returns the grid map
void Map::getGridMap(GridMap *g) {

    // copy, is it really necessary?
    // lock the mutex
    map_mutex.lock();

    if (!map_copy) {

        // copy the entire grid map
        g->copy(grid);

        map_copy = true;

    }

    // lock the mutex
    map_mutex.unlock();

}

//To ray cast 
void Map::getMap(nav_msgs::OccupancyGrid *m) {

    // copy, is it really necessary?
    // lock the mutex
    map_mutex.lock();

    if (!map_copy) {

        // copy the entire grid map
        *m = map;

        map_copy = true;

    }

    // lock the mutex
    map_mutex.unlock();

}

// force map update
void Map::forceUpdate() {

    // lock the map
    map_mutex.lock();

    // set to false
    map_received = false;
    map_copy = false;

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
void Map::updateAvailableCells() {

    if(!availableCells.empty()) {
        availableCells.clear();
    }

    double size = map.info.width*map.info.height;

    // get all available cells
    for (int i = 0; i < size; i++) {

        if (0.65 <= map.data[i]) {
            availableCells.push_back(i);
        }
    }
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

        // set a uniform distribution
        std::uniform_int_distribution<int> uniform_dist(0, availableCells.size() - 1);

        // tmp variables
        int index;
        double *pose;
        int g_i, g_j;
        float div = 1.0/(float)grid.width;

        // shortcut
        Sample2D *samples = Xt->samples;

        // sledgehammer programing style? =-)
        // create random poses based on the unnoccupied cells
        for (int i = 0; i < Xt->size; i++) {

            // a simple pointer to avoid repetitive writing, another shortcut
            pose = samples[i].pose.v;

            // get a random index from the uniform_int_distribution
            index = uniform_dist(generator);

            // get the grid x coord
            g_i = availableCells[index]%grid.width;

            // get the grid y coord - div is defined above as 1/grid.width
            g_j = availableCells[index]*div;

            // now we have the grid index and a we can 
            // convert to world coords and assign to the pose value
            // with a simple gaussian noise
            pose[0] = (grid.origin_x + (g_i - grid.width/2)*grid.scale) + normal_dist(generator);
            pose[1] = (grid.origin_y + (g_j - grid.height/2)*grid.scale) + normal_dist(generator);

            // random orientation between 0 and 2*PI radians
            pose[2] = angle_dist(generator);

        }

        // spreaded!
        Xt->spreaded = true;
    }

     std::cout << "Spreaded!" << std::endl;

    // unlock the map
    map_mutex.unlock();

}

// generates random pose
Pose2D Map::randomPose2D() {

    // a new pose to return
    Pose2D pose;

    // lock the map
    map_mutex.lock();

    // set a uniform distribution
    std::uniform_int_distribution<int> uniform_dist(0, availableCells.size() - 1);

    // get the index to acess the availableCells
    int index = uniform_dist(generator);

    // get the grid x coord
    int g_i = availableCells[index]%grid.width;

    // get the grid y coord - div is defined above as 1/grid.width
    int g_j = availableCells[index]/grid.width;

    // now we have the grid index and a we can 
    // convert to world coords and assign to the pose value
    // with a simple gaussian noise
    pose.v[0] = (grid.origin_x + (g_i - grid.width/2)*grid.scale) + normal_dist(generator);
    pose.v[1] = (grid.origin_y + (g_j - grid.height/2)*grid.scale) + normal_dist(generator);

    // random orientation between 0 and 2*PI radians
    pose.v[2] = angle_dist(generator);

    // unlock the map
    map_mutex.unlock();

    return pose;

}
