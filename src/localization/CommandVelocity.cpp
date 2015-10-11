#include "CommandVelocity.hpp"

// basic constructor
CommandVel::CommandVel() : cmds({}), last_cmd({}), start(), end(), cmds_mutex() {
    // set the last current time
    last_cmd.stamp = ros::Time::now();
}
// push a new message
void CommandVel::push_back(const geometry_msgs::Twist &msg, unsigned int msg_seq) {

    // lock the mutex
    cmds_mutex.lock();

    // creates a new Velocity
    Velocity v;

    // updates the TwistStamped
    v.linear = msg.linear.x;
    v.angular = msg.angular.z;

    // updates the header
    v.seq = msg_seq;
    v.stamp = ros::Time::now(); // not so good... :-(

    // push to the vector
    cmds.push(v);

    // unlock the mutex
    cmds_mutex.unlock();
}

// get the entire vector
// it stops at the last command before the current laser
std::vector<Velocity> CommandVel::getAll() {

    // lock the mutex
    cmds_mutex.lock();

    // creates a new std::vector
    std::vector<Velocity> commands;

    // get the last command
    // let the last_cmd be the command that starts at the last LaserScan timestamp
    commands.push_back(last_cmd);

    // copy the entire vector and clear the cmds
    while(!cmds.empty() && cmds.front().header.stamp < end) {
        commands.push_back(cmds.front());
        cmds.pop();
    }

    // set the last_cmd
    last_cmd = commands.back();
    // updates the timestamp to the new LaserScan timestamp
    last_cmd.header.stamp = end;

    // unlock the mutex
    cmds_mutex.unlock();

    return commands;

}

// updates the current limit time
void CommandVel::setTimeLimits(const ros::Time &time) {

    // lock the mutex
    cmds_mutex.lock();

    // updates the limit time
    start = end;
    end = time;

    // unlock the mutex
    cmds_mutex.unlock();

}