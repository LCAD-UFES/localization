#ifndef COMMAND_VELOCITY_H
#define COMMAND_VELOCITY_H

#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

class CommandVel {
    private:
        // the Twist message queue
        std::queue<geometry_msgs::TwistStamped> cmds;
        // the current limit time
        ros::Time limit;

        // the mutex
        std::mutex cmds_mutex;
    public:
        // push a new message
        void push_back(const geometry_msgs::Twist&, unsigned int);
        // get the entire vector and clear
        std::vector<geometry_msgs::TwistStamped> getAll();
        // update the current limit time
        void setLimitTime(const ros::Time&);

};

#endif