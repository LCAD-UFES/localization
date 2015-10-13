#ifndef LASER_H
#define LASER_H

#include <mutex>

#include "sensor_msgs/LaserScan.h"

#include "Scan.hpp"

class Laser {
    private:

        // the laser scan data - see Scan.cpp
        // it's our internal representation
        Scan ls_scan;

        // locks the ls_scan
        std::mutex ls_mutex;

    public:
        // returns the laser scan
        void getScan(Scan*);

        // updates the laser scan
        void setScan(const sensor_msgs::LaserScan&);
};

#endif