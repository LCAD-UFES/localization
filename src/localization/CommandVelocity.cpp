#include "CommandVelocity.hpp"

// basic constructor
CommandVel::CommandVel(const geometry_msgs::TwistStamped &msg) : CommandReader(), cmd() {
    cmd = msg;
    std::cout << "SEQ: " << msg.header.seq << std::endl;
    std::cout << "TIME: " << msg.header.stamp << std::endl;
    std::cout << "Frame ID: " << msg.header.frame_id << std::endl;
}

// basic constructor
CommandVel::CommandVel(const geometry_msgs::Twist &msg, unsigned int seq) : CommandReader(), cmd() {

    // updates the Twist
    cmd.twist = msg;
    // updates the header
    cmd.header.seq = seq;
    cmd.header.stamp = ros::Time::now(); // not so good... :-(
    cmd.header.frame_id = "/base_link";
    std::cout << "SEQ: " << cmd.header.seq << std::endl;
    std::cout << "TIME: " << cmd.header.stamp << std::endl;
    std::cout << "Frame ID: " << cmd.header.frame_id << std::endl;
}