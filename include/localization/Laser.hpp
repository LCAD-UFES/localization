#ifndef LASER_H
#define LASER_H

#include <mutex>

#include "sensor_msgs/LaserScan.h"

#include "Scan.hpp"

class Laser {
    private:

        //need to Beam Model Ray Cast - provisionally
        sensor_msgs::LaserScan laser_msg;

        // locks the ls_scan
        std::mutex ls_mutex;

    public:

        // updates the laser scan
        void setLaserScan(const sensor_msgs::LaserScan&);

        // returns the laser scan
        void getScan(Scan*);

        //to Beam Model - Provisionally
        void getLaserScan(sensor_msgs::LaserScan*);



};

#endif
