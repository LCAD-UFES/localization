#include <iostream>

#include "Map.hpp"

// basic constructor
Map::Map(const ros::NodeHandle &private_nh) : grid(private_nh), normal_dist(0.0, 0.05), generator(std::random_device {} ()), angle_dist(-M_PI, M_PI) {

    // measurement model
    std::string measurement_model;
    private_nh.param<std::string>("measurement_model", measurement_model, "likelihood");

    if (0 == measurement_model.compare("beam")) {
        likelihood_flag = false;
    } else {
        likelihood_flag = true;
    }
}

// receives a OccupancyGrid msg and converts to internal representation
void Map::updateMap(const nav_msgs::OccupancyGrid &map_msg) {

    // update status

    // lock the mutex
    map_mutex.lock();

    // save the OccupancyGrid to our object container
    grid.updateGridMap(map_msg);

    // update the availableCells
    updateAvailableCells();

    if (likelihood_flag) {
        buildGridMap();
    }
    // unlock the mutex
    map_mutex.unlock();

}

// private method
void Map::buildGridMap() {

    // updates the likelihood
    grid.nearestNeighbor();

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

// get the cells pointer
void Map::updateAvailableCells() {

    if(!availableCells.empty()) {
        availableCells.clear();
    }

    // get all available cells
    for (int i = 0; i < grid.size; i++) {

        if (0 <= grid.cells[i].occ_state && 0.65 > grid.cells[i].occ_state) {
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
    if (10 < availableCells.size()) {

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
            pose[0] = (grid.origin_x + (g_i - (grid.width >> 1))*grid.resolution) + normal_dist(generator);
            pose[1] = (grid.origin_y + (g_j - (grid.height >> 1))*grid.resolution) + normal_dist(generator);

            // random orientation between 0 and 2*PI radians
            pose[2] = angle_dist(generator);

        }

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
    pose.v[0] = (grid.origin_x + (g_i - grid.width/2)*grid.resolution) + normal_dist(generator);
    pose.v[1] = (grid.origin_y + (g_j - grid.height/2)*grid.resolution) + normal_dist(generator);

    // random orientation between 0 and 2*PI radians
    pose.v[2] = angle_dist(generator);

    // unlock the map
    map_mutex.unlock();

    return pose;

}

void Map::export_grid_map(nav_msgs::OccupancyGrid &occupancy) {

    // time stamp
    occupancy.header.stamp = ros::Time::now();

    // frame_id
    occupancy.header.frame_id = "map";

    // the meta data
    occupancy.info.map_load_time = ros::Time::now();

    // resolution
    occupancy.info.resolution = grid.resolution;

    // width
    occupancy.info.width = grid.width;

    // height
    occupancy.info.height = grid.height;

    // origin - position
    occupancy.info.origin.position.x = grid.origin_x - grid.width2*grid.resolution;
    occupancy.info.origin.position.y = grid.origin_y - grid.width2*grid.resolution;
    occupancy.info.origin.position.z = 0.0;

    // origin - orientation
    occupancy.info.origin.orientation.x = 0.0;
    occupancy.info.origin.orientation.y = 0.0;
    occupancy.info.origin.orientation.z = 0.0;
    occupancy.info.origin.orientation.w = 1.0;

    // resize the occupancy data
    occupancy.data.resize(grid.size);

    // copy the cells
    MapCell *cells = grid.cells;

    for (int i = 0; i < grid.size; i++) {

        if (10 < cells[i].occ_state) {

            occupancy.data[i] = 100;

        } else if (-10 > cells[i].occ_state) {

            occupancy.data[i] = 0;

        } else {

            occupancy.data[i] = -1;

        }

    }

}
