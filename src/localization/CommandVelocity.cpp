#include "CommandVelocity.hpp"

// push a new message
void CommandVel::push_back(const geometry_msgs::Twist &msg, unsigned int msg_seq) {

    // lock the mutex
    cmds_mutex.lock();

    // creates a new TwistStamped
    geometry_msgs::TwistStamped twist_stamped;

    // updates the TwistStamped
    twist_stamped.twist = msg;
    // updates the header
    twist_stamped.header.seq = msg_seq;
    twist_stamped.header.stamp = ros::Time::now(); // not so good... :-(
    twist_stamped.header.frame_id = "/base_link";

    // push to the vector
    cmds.push(twist_stamped);

    // updates the limit
    limit = twist_stamped.header.stamp;

    // unlock the mutex
    cmds_mutex.unlock();
}

// get the entire vector
std::vector<geometry_msgs::TwistStamped> CommandVel::getAll() {

    // lock the mutex
    cmds_mutex.lock();

    // creates a new std::vector
    std::vector<geometry_msgs::TwistStamped> commands;

    // copy the entire vector and clear the cmds
    while(!cmds.empty() && cmds.front().header.stamp < limit) {
        commands.push_back(cmds.front());
        cmds.pop();
    }
    // unlock the mutex
    cmds_mutex.unlock();

    return commands;
}

// updates the current limit time
void CommandVel::setLimitTime(const ros::Time &time) {

    // lock the mutex
    cmds_mutex.lock();

    // updates the limit time
    limit = time;

    // unlock the mutex
    cmds_mutex.unlock();
}