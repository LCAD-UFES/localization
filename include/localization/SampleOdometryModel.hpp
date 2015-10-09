#ifndef SAMPLE_ODOMETRY_MODEL_H
#define SAMPLE_ODOMETRY_MODEL_H

#include "CommandOdometry.hpp"
#include "SampleMotionModel.hpp"

#include "ros/ros.h"

class SampleOdometryModel : public SampleMotionModel {

    // parameters
    double a1, a2, a3, a4;

    // the command
    /* TODO */

    public:
        // constructor
        SampleOdometryModel(const ros::NodeHandle&, std::vector<CommandReader *>);
        // you probably will need to provide a destructor to avoid duplicated deletes over the Commmand pointer

        // updates the pose to a new one based on the CommandVel
        void samplePose2D(Pose2D *pose);
};

#endif