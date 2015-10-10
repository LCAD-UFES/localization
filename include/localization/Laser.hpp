#ifndef LASER_H
#define LASER_H

#include <mutex>

#include "sensor_msgs/LaserScan.h"

class Laser {
    private:
        // the processed scan data
        sensor_msgs::LaserScan ls_scan;
        // locks the ls_scan
        std::mutex ls_mutex;

    public:
        // returns the laser scan
        sensor_msgs::LaserScan getScan();
        // updates the laser scan
        void setScan(const sensor_msgs::LaserScan&);
};

#endif