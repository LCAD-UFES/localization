#ifndef COMMAND_ODOMETRY_H
#define COMMAND_ODOMETRY_H

#include "Pose2D.hpp"
#include <ros/ros.h>

class CommandOdom {

    // the old and new poses are set by the ParticleFilter callback method
    Pose2D old_pose, new_pose;

    public:
        
};
#endif