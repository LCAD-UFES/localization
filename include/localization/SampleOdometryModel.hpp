#ifndef SAMPLE_ODOMETRY_MODEL_H
#define SAMPLE_ODOMETRY_MODEL_H

#include "CommandOdometry.hpp"
#include "SampleMotionModel.hpp"
#include "wrap2pi.h"
#include "Pose2D.hpp"
#include "std_msgs/String.h"
#include <vector>

class SampleOdometryModel : public SampleMotionModel {

    // parameters
    double alpha1, alpha2, alpha3, alpha4;

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

        // updates the pose to a new one based on the CommandOdom
        virtual void samplePose2D(Pose2D*);

        // updates the commands
        virtual void update(const ros::Time&);
        double angleDiff(double a, double b);
        double normalize(double a);
};

#endif
