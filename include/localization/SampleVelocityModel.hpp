#ifndef SAMPLE_MOTION_MODEL_VELOCITY_H
#define SAMPLE_MOTION_MODEL_VELOCITY_H

#include "SampleMotionModel.hpp"

#include "ros/ros.h"

class SampleVelocityModel : public SampleMotionModel {

    // the command
    ros::geometry_msgs::Twist cmd;

    // Sample Velocity Model parameters 
    double a1, a2, a3, a4, a5, a6;
    double deltaT;

    // the subscriber
    ros::Subscriber sub;

    public:
        // default constructor
        SampleVelocityModel(const ros::NodeHandle&, const ros::NodeHandle&);
        // constructor

        // updates the pose to a new one based on the command
        virtual void samplePose2D(Pose2D *pose);
        // the callback function to listen the command topic (e.g /cmd_vel)
        void(const geometry_msgs::Twist msg);

};

#endif