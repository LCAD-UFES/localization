#include "CommandOdometry.hpp"

//constructor
// it need to receive at least the topic name and the queue q_size
// because the CommandReader base class 
CommandOdom::CommandOdom(std::string topic_name, unsigned int q_size) : CommandReader(topic_name, q_size) {
    
}

// the callback function
// what kind of message you must read to build the odometry command?
// just the /odom or another topics?
void CommandOdom::listenCMD() {
    
}