#ifndef COMMAND_VELOCITY_H
#define COMMAND_VELOCITY_H

#include "CommandReader.hpp"
#include <geometry_msgs/Twist.h>

#include "ros/ros.h"

class CommandVel : public CommandReader {
    private:
        // the Twist message
        geometry_msgs::Twist last_msg;

    public:
        // base constructor
        CommandVel(const ros::NodeHandle&);
        // constructor
        CommandVel(std::string, unsigned int);
        // read the apropriate velocity from the correct ROS topic
        void listenCMD(const geometry_msgs::Twist);
        // translational and rotational speeds
        double linear, angular;
};

#endif