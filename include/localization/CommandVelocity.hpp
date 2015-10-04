#ifndef COMMAND_VELOCITY_H
#define COMMAND_VELOCITY_H

#include "CommandReader.hpp"
#include <geometry_msgs/Twist.h>

class CommandVel : public CommandReader {
    private:
        // translation and rotation speeds
        double linear, angular;
        // the Twist message
        geometry_msgs::Twist last_msg;

        // read the apropriate velocity from the correct ROS topic
        void listen(const geometry_msgs::Twist);

    public:
        // constructor
        CommandVel(std::string, unsigned int);
        // read the topic
        void read();

};

#endif