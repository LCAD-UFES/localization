#include "Laser.hpp"

// updates the laser scan
void Laser::setLaserScan(const sensor_msgs::LaserScan &msg) {

    // lock the mutex
    ls_mutex.lock();

    // save the LaserScan message
    ls_scan = msg;

    // unlock the mutex
    ls_mutex.unlock();

}
// get the entire laser scan
void Laser::getLaserScan(sensor_msgs::LaserScan &ls){

    // lock the mutex
    ls_mutex.lock();

    // copy the LaserScan to our BeamRangeFinderModel
    ls = ls_scan;

    // unlock the mutex
    ls_mutex.unlock();

}
