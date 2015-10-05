#ifndef LASER_READER_H
#define LASER_READER_H

#include "SensorReader.hpp"
#include "sensor_msgs/LaserScan.h"

class LaserReader : public Sensor {
    private:
        // the scan data
        sensor_msgs::LaserScan scan;
    public:
};

#endif