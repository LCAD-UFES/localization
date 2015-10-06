#ifndef MONTE_CARLO_LOCALIZATION_H
#define MONTE_CARLO_LOCALIZATION_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
#include "SampleSet.hpp"

class MCL {
    private:
        // the Node 
        // the ros NodeHandle
        ros::NodeHandle nh;
        // the /laser/scan subscriber

        // The set of samples
        SampleSet Xt;

    public:
        // this constructor receives the amount of samples
        // it needs to specify the velocity and measurement models
        // Velocity Models: "vel" == "velocity" or "odom" == odometry
        // Measurement Models: "beam" == beam range find or "likelyhood" == likelyhood field
        // and finally the map filename as the last argument
        MCL(unsigned int, std::string, std::string);

        // or you can pass the the Models
        MCL(unsigned int, SampleMotionModel*, MeasurementModel*);

        // the run method
        void run(const sensor_msgs::LaserScan);
};

#endif