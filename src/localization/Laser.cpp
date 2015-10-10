#include "Laser.hpp"

// returns the laser scan
sensor_msgs::LaserScan Laser::getScan() {

    // copy, is it really necessary???

    // lock the mutex
    ls_mutex.lock();
    // acces the ls_scan
    sensor_msgs::LaserScan ls = ls_scan;
    // unlock the mutex
    ls_mutex.unlock();

    return ls;
}

// updates the laser scan
void Laser::setScan(const sensor_msgs::LaserScan &ls) {

    // lock the mutex
    ls_mutex.lock();
    // acces the ls_scan
    ls_scan = ls;
    // unlock the mutex
    ls_mutex.unlock();

}