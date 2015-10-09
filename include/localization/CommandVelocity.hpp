#ifndef COMMAND_VELOCITY_H
#define COMMAND_VELOCITY_H

#include "CommandReader.hpp"

#include "ros/ros.h"

class CommandVel : public CommandReader {
    private:
        // the Twist message
        std::vector<geometry_msgs::Twist> cmds;

    public:

        // get the command
        virtual geometry_msgs::TwistStamped getCMD();
        virtual void setCMD(geometry_msgs::TwistStamped);
};

#endif