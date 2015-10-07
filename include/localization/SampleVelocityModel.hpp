#ifndef SAMPLE_MOTION_MODEL_VELOCITY_H
#define SAMPLE_MOTION_MODEL_VELOCITY_H

#include "SampleMotionModel.hpp"

#include "ros/ros.h"
#include "ros/spinner.h"
#include "ros/callback_queue.h"
#include "geometry_msgs/Twist.h"

class SampleVelocityModel : public SampleMotionModel {

    // the command
    ros::geometry_msgs::Twist last_cmd;

    // Sample Velocity Model parameters 
    double a1, a2, a3, a4, a5, a6;
    double deltaT;

    // This class custom CallabackQueue
    ros::CallbackQueue cmd_queue;
    // the Asyncronous Spinner
    ros::AsyncSpinner spinner;
    // the Subscriber
    ros::Subscriber sub;
    // the Subscribe Options
    ros::SubscribeOptions ops;

    public:
        // default constructor
        SampleVelocityModel(const ros::NodeHandle&, const ros::NodeHandle&);
        // constructor
        // updates the pose to a new one based on the command
        virtual void samplePose2D(Pose2D *pose);
        // the callback function to listen the command topic (e.g /cmd_vel)
        void listenCMD(const geometry_msgs::TwistStamped msg);

};

#endif