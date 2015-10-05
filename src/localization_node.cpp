#include "CommandVelocity.hpp"
#include "Sample2D.hpp"

int main(int argc, char **argv) {
    // ROS
    ros::init(argc, argv, "localization");

    // 
    CommandVel cmd("cmd_vel", 100);
//     SampleSet set;

    Sample2D s;
    s.weight = 0.4;

    while(ros::ok()) {
        ros::spin();
    }
    return 0;
}