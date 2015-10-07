#ifndef MONTE_CARLO_LOCALIZATION_H
#define MONTE_CARLO_LOCALIZATION_H

#include <string>

#include "ros/ros.h"

#include "Laser.hpp"
#include "SampleSet.hpp"
#include "MonteCarloParameters.hpp"

class MCL {
    private:
        // the Node 
        // the ros NodeHandle
        ros::NodeHandle nh;
        // the private NodeHandle
        ros::NodeHandle private_nh;
        // The LaserScan object
        // should Implements all laser methods
        Laser laser;
        // The set of samples
        SampleSet Xt;
        // The laser/Scan subscriber
        ros::Subscriber ls_sub;

    public:
        MCL();

        // the run method
        void run(const sensor_msgs::LaserScan::ConstPtr& scan_in);
};

#endif