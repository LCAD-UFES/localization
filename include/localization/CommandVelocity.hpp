#ifndef COMMAND_VELOCITY_H
#define COMMAND_VELOCITY_H

#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "Velocity.hpp"

class CommandVel {

    private:

        // the Twist message queue
        std::queue<Velocity> cmds;

        // just to mark the TwistStamped seq
        unsigned int msg_seq;

        // the last processed cmd
        Velocity last_cmd;

        // the current time limit
        ros::Time start;
        ros::Time end;

        // the mutex
        std::mutex cmds_mutex;

    public:
        // basic constructor
        CommandVel();

        // push a new message
        void push_back(const geometry_msgs::Twist&);

        // get the entire vector and clear
        std::vector<Velocity> getAll();

        // update the current time limits
        void setTimeLimits(const ros::Time&);

};

#endif