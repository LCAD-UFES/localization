#include "CommandVelocity.hpp"

CommandVel::CommandVel(std::string topic_name, unsigned int q_size) : CommandReader(topic_name, q_size) {
    // subscribe to the correct topic
    // the subscribe function below accepts the public method address and and a pointer
    // "this"
    // overloaded subscribe method provided by ROS team
    // see the listenCMD method below
    sub = nh.subscribe(topic, queue_size, &CommandVel::listenCMD, this);
}

// the callback method to listen the command 
// in this case it gets a Twist message and assigns to last_msg private attribute
// we need to implement a real world solution
// for now, it's just to start and learn how to use Object methods as a Subscriber callback
void CommandVel::listenCMD(const geometry_msgs::Twist msg) {
    last_msg = msg;
}
