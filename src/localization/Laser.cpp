#include "Laser.hpp"

// returns the entire LaserScan
// copy, is it really necessary???
void Laser::getScan(Scan *s) {

    // lock the mutex
    ls_mutex.lock();

    // our internal LaserScan representation
    s->copy(ls_scan);

    // unlock the mutex
    ls_mutex.unlock();

}

// updates the laser scan
void Laser::setScan(const sensor_msgs::LaserScan &msg) {

    // lock the mutex
    ls_mutex.lock();

    // update the Scan with a new ros LaserScan message
    ls_scan.updateScan(msg);

    // unlock the mutex
    ls_mutex.unlock();

}