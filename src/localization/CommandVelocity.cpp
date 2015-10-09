#include "CommandVelocity.hpp"

// get the command
geometry_msgs::TwistStamped CommandVelocity::getCMD() {
    return last_cmd;
}

virtual void setCMD(geometry_msgs::TwistStamped msg) {
    last_cmd = msg;
}