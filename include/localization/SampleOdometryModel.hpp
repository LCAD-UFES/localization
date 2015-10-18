#ifndef SAMPLE_ODOMETRY_MODEL_H
#define SAMPLE_ODOMETRY_MODEL_H

#include "CommandOdometry.hpp"
#include "SampleMotionModel.hpp"
#include "wrap2pi.h"
#include "Pose2D.hpp"

class SampleOdometryModel : public SampleMotionModel {

    // parameters
    double a1, a2, a3, a4;

    // the CommandOdom reader (?)
    CommandOdom *cmds;
    //the command
    std::vector<Pose2D> commands;
    //the command xt-1
    Pose2D old_odom;
    //the command xt
    Pose2D odom;

    public:
        // constructor
        SampleOdometryModel(const ros::NodeHandle&, CommandOdom *);
        // you probably will need to provide a destructor to avoid duplicated deletes over the Commmand pointer
        ~SampleOdometryModel();

        // updates the pose to a new one based on the CommandVel
        virtual void samplePose2D(Pose2D*);

        // updates the commands
        virtual void update(const ros::Time&);
        double SampleOdometryModel::angleDiff(double a, double b);
        double SampleOdometryModel::normalize(double a);
};

#endif
