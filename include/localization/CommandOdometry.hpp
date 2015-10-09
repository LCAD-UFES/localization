#ifndef COMMAND_ODOMETRY_H
#define COMMAND_ODOMETRY_H

#include <ros/ros.h>

#include "Pose2D.hpp"
#include "CommandReader.hpp"

class CommandOdom : public CommandReader {

    public:
        // the old and new poses are set by the ParticleFilter callback method
        Pose2D old_pose, new_pose;
};
#endif