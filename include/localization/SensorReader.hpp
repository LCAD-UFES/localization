#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include "ros/ros.h"

class SensorReader {
    private:
        // the node Handler
        ros::NodeHandle nh;
        // the topic subscriber
        ros::Subscriber sub;
    public:
};


#endif