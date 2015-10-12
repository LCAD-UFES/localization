#ifndef SCAN_H
#define SCAN_H

#include <ros/ros.h>

template<int size>
struct Scan {
    public:
        // start angle of the scan
        float angle_min;
        // end angle of the scan
        float angle_max;
        // angular distance between measurements
        float angle_increment;
        // time betwen scans
        float time_increment;

        // mininum range value
        float range_min;
        // max range value
        float range_max;

        // range data
        float ranges[size];

        // the LaserScan time, just to sync with the commands
        ros::Time time;
};

typedef Scan<60> LS_60;

#endif