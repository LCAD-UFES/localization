#ifndef LASER_H
#define LASER_H

#include <mutex>

#include "sensor_msgs/LaserScan.h"

#include "Scan.hpp"

class Laser {
    private:
        // the laser scan data
        LS_60 ls_scan;

        // locks the ls_scan
        std::mutex ls_mutex;

    public:
        // returns the laser scan
        void getScan(LS_60 *);

        // updates the laser scan
        void setScan(const sensor_msgs::LaserScan&);
};

#endif