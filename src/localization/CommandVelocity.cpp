#include "CommandVelocity.hpp"

// basic constructor
CommandVel::CommandVel(const geometry_msgs::TwistStamped &msg) : cmd(msg) {}