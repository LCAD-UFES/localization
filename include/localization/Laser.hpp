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

        //need to Beam Model Ray Cast - provisionally
        sensor_msgs::LaserScan laser_msg;
        // locks the ls_scan
        std::mutex ls_mutex;

    public:
        // returns the laser scan
        void getScan(Scan*);

        // updates the laser scan
        void setScan(const sensor_msgs::LaserScan&);

        //to Beam Model - Provisionally
        const sensor_msgs::LaserScan getMsgScan();


};

#endif
