#include "Laser.hpp"

// updates the laser scan
void Laser::setLaserScan(const sensor_msgs::LaserScan &msg) {

    // lock the mutex
    ls_mutex.lock();

    // save the LaserScan message
    laser_msg = msg;

    // unlock the mutex
    ls_mutex.unlock();

}
//To ray cast - provisionally
void Laser::getLaserScan(sensor_msgs::LaserScan *ls){

    // lock the mutex
    ls_mutex.lock();

    // copy the LaserScan to our BeamRangeFinderModel
    *ls = laser_msg;

    // unlock the mutex
    ls_mutex.unlock();

}

// returns the entire LaserScan
// copy, is it really necessary???
void Laser::getScan(Scan *s) {

    // lock the mutex
    ls_mutex.lock();

    // our internal LaserScan representation
    s->updateScan(laser_msg);

    // unlock the mutex
    ls_mutex.unlock();

}