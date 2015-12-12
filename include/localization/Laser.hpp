#ifndef LASER_H
#define LASER_H

#include <mutex>

#include "sensor_msgs/LaserScan.h"

class Laser {

    private:

        sensor_msgs::LaserScan ls_scan;

        // locks the ls_scan
        std::mutex ls_mutex;

    public:

        // updates the laser scan
        void setLaserScan(const sensor_msgs::LaserScan&);

        //
        void getLaserScan(sensor_msgs::LaserScan&);

};

#endif
