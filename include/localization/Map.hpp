#ifndef MAP_H
#define MAP_H

#include <mutex>

#include "nav_msgs/OccupancyGrid.h"

#include "SampleSet.hpp"
#include "GridMap.hpp"

class Map {
    private:

        // our internal GridMap/OccupancyGrid representation
        GridMap grid;

        // mutex to lock map
        std::mutex map_mutex;

        // flag to avoiding unnecessary copies
        bool map_received;

        // 
        bool grid_copy;

        // get the free cells
        std::vector<int> availableCells;

        // update the availableCells
        void updateAvailableCells();

        // random 
        // set another uniform distribution -  0 ~ 2PI
        std::uniform_real_distribution<double> angle_dist;

        // set a normal distribution
        std::normal_distribution<double> normal_dist;

        // set a generator engine
        std::default_random_engine generator;

        //to beam model ray cast - provisionally
        nav_msgs::OccupancyGrid mapaMsg;

    public:


        Map();
        // updates the grid
        bool updateMap(const nav_msgs::OccupancyGrid&);

        // update max_occ_dist
        void updateMaxOccDist(double);

        // returns the grid
        void getGridMap(GridMap *g);

        // returns the map flag
        bool mapReceived();

        // force map update
        void forceUpdate();

        // spreads the particles over the entire map, randomly
        void uniformSpread(SampleSet*);

        // returns an random Pose2D inside the available cells
        Pose2D randomPose2D();
        //To ray cast - provisionally
        const nav_msgs::OccupancyGrid getMsgMap();


};

#endif
