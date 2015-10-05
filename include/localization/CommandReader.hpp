#ifndef COMMAND_READER_H
#define COMMAND_READER_H

#include <string>
#include "ros/ros.h"

class CommandReader {
    protected:
        // the ros topic name
        std::string topic;
        // the queue size
        unsigned int queue_size;
        // the node handler
        ros::NodeHandle nh;
        // it needs to subscribe to the correct ROS topic
        // so I make it a pointer here, you should instantiate a subscriber to the correct topics
        // based on the type of command used in your models
        // Example: the Velocity Motion Model uses a command u(v,w)
        ros::Subscriber sub;
    public:
        // basic constructor
        CommandReader(std::string topic_name, unsigned int);
};

#endif