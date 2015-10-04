#include "CommandVelocity.hpp"

#include <iostream>

CommandVel::CommandVel(string topic_name, unsigned int q_size) : CommandReader(topic_name, q_size) {
    sub = nh.subscribe(topic, queue_size, this.read);
}

void CommandVel::listen(const geometry_msgs::Twist msg) {
    last_msg = msg;
    linear = msg->linear.x;
    angular = msg->angular.z;
}

void CommandVel::read() {
    
}