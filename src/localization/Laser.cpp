#include "Laser.hpp"

// returns the entire LaserScan
// copy, is it really necessary???
void Laser::getScan(LS_60 *ls) {

    // lock the mutex
    ls_mutex.lock();

    *ls = ls_scan;

    // unlock the mutex
    ls_mutex.unlock();

}

// updates the laser scan
void Laser::setScan(const sensor_msgs::LaserScan &ls) {

    // lock the mutex
    ls_mutex.lock();

    // copy the basic info
    // start angle of the scan
    ls_scan.angle_min =  ls.angle_min;
    // end angle of the scan
    ls_scan.angle_max = ls.angle_max;
    // angular distance between measurements
    ls_scan.angle_increment = ls.angle_increment;
    // time betwen scans
    ls_scan.time_increment  = ls.time_increment;

    // mininum range value
    ls_scan.range_min = ls.range_min;
    // max range value
    ls_scan.range_max = ls.range_max;

    // range data
    for (int i = 0; i < ls.ranges.size(); i++) {

        // copy the range value
        if (ls.range_min < ls.ranges[i]) {
            ls_scan.ranges[i][0] = ls.ranges[i];
        } else {
            ls_scan.ranges[i][0] = ls.range_max;
        }

        // the angle value
        ls_scan.ranges[i][1] = ls.angle_min + (i * ls.angle_increment);
    }

    // the range count
    ls_scan.range_count = ls.ranges.size();

    // save the LaserScan timestamp
    ls_scan.time = ls.header.stamp;

    // unlock the mutex
    ls_mutex.unlock();

}