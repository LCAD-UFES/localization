#include "CommandVelocity.hpp"

// basic constructor
CommandVel::CommandVel(const geometry_msgs::TwistStamped &msg) : CommandReader(), cmd() {
    cmd = msg;
}

// basic constructor
CommandVel::CommandVel(const geometry_msgs::Twist &msg, unsigned int seq) : CommandReader(), cmd() {

    // updates the Twist
    cmd.twist = msg;
    // updates the header
    cmd.header.seq = seq;
    cmd.header.stamp = ros::Time::now(); // not so good... :-(
    cmd.header.frame_id = "/base_link";
}