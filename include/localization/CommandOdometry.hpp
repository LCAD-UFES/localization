#ifndef COMMAND_ODOMETRY_H
#define COMMAND_ODOMETRY_H

#include "CommandReader.hpp"
#include "Pose2D.hpp"

class CommandOdom : public CommandReader {
    private:
        /* TODO */
        // this object must to subscribe the odometry/tf/cmd_vel topics
        // and then you can use some 
        // and assign the old and new pose to the public attributes below
        // see the CommandVel implementation
    
        // the old and new poses are set by the your callback method
        // they are updated from the ROS topic
        // this object should subscribe the /odom topic
        // and read the informations from there
        Pose2D old_pose, new_pose;
    public:
};
#endif