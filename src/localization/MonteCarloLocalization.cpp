#include "MonteCarloLocalization.hpp"

MCL::MCL() : nh(), private_nh("~"), laser(), Xt(private_nh) {

    // set the Laser pointer inside the Measurement Model
    Xt.measurement->setLaser(&laser);

    // get the laser topic parameter
    std::string laser_topic;
    nh.param<std::string>("laser_scan_topic", laser_topic, "/p3dx/laser/scan");

    // subscribe to the laser topic (eg. /p3dx/laser/scan)
    ls_sub = nh.subscribe(laser_topic, 10, &MCL::run, this);

}

void MCL::run(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    // process the Laser scan
    // the Laser object is also available to the Measurement Model
    laser.toCloud(scan_in);

    // sample the entire set! Get new pose from previous pose and command
    // and also updates the poses weights from the measurement model
    // see the SampleSet class
    Xt.sample();

    // the resample algorithm
    Xt.resample();

    // now the Xt is a set of a new bel(xt)
    // the MCL algorithm usually returns the updated set:
    // return Xt;
    // But this is a callback function
    // so we can just publish Xt to a topic...
    // e.g pub.publish();
    /* TODO */
}
