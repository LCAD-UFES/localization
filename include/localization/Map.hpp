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

        // flag to avoiding unnecessary copies
        bool map_copy;

        // flag to build the gridMap and the likelihood
        bool built;
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
        nav_msgs::OccupancyGrid map;

        // build the likelihood
        void buildGridMap();

    public:

        // basic constructor
        Map();

        // updates the map
        bool updateMap(const nav_msgs::OccupancyGrid&);

        // update max_occ_dist
        void updateMaxOccDist(double);


        // returns the grid
        void getGridMap(GridMap *);

        //To ray cast - provisionally
        void getMap(nav_msgs::OccupancyGrid *);

        // returns the map flag
        bool mapReceived();

        // force map update
        void forceUpdate();

        // spreads the particles over the entire map, randomly
        void uniformSpread(SampleSet*);

        // returns an random Pose2D inside the available cells
        Pose2D randomPose2D();


};

#endif
