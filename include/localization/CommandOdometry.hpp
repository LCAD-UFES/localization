#ifndef COMMAND_ODOMETRY_H
#define COMMAND_ODOMETRY_H

#include "CommandReader.hpp"
#include "pose.hpp"

class CommandOdom : public CommandReader {
    /* TODO */
    // this object must to subscribe the odometry/tf/cmd_vel topics
    // and then you can use the read() method to obtain all the apropriate infos from the topics
    // and assign the old and new pose to the public attributes below
    
    public:
        //constructor
        // it need to receive at least the topic name and the queue q_size
        // because the CommandReader base class 
        CommandOdom(std::string topic_name, unsigned int q_size);
        // read or build the odometry model command from the ROS topics
        void read();

        // the old and new poses are set by the read() method
        // they are updated from the ROS topic
        // this object should subscribe the /odom topic
        // and read the informations from there
        Pose2D old_pose, new_pose;
};
#endif