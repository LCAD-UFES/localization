#ifndef VELOCITY_H
#define VELOCITY_H

#include <ros/ros.h>

struct Velocity {
    // ROS like header
    unsigned int seq;
    ros::Time stamp;

    double linear;
    double angular;
};

#endif
