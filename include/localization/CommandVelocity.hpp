#ifndef COMMAND_VELOCITY_H
#define COMMAND_VELOCITY_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "CommandReader.hpp"

class CommandVel : public CommandReader {
    public:
        // basic constructor
        CommandVel(const geometry_msgs::TwistStamped&);

        // basic constructor 2
        CommandVel(const geometry_msgs::Twist&, unsigned int);

        // the Twist message
        geometry_msgs::TwistStamped cmd;
};

#endif