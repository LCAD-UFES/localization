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

        // get the free cells
        std::vector<int> getAvailableCellsIndexes();

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

};

#endif