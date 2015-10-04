#ifndef COMMAND_VELOCITY_H
#define COMMAND_VELOCITY_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <cstdlib>
#include <string>

class CommandVel {
    private:
        // translation and rotation speeds
        double linear, angular;
        // the Twist message
        geometry_msgs::Twist last_msg;
    public:
        // constructor
        CommandVel(String, unsigned int);
        // read the apropriate velocity from the correct ROS topic
        void listen(const geometry_msgs::Twist);
        // read the topic
        void read();

};

#endif