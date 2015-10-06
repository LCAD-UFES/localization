#include "MonteCarloLocalization.hpp"

int main(int argc, char **argv) {
    // ROS
    ros::init(argc, argv, "localization");

    // MonteCarlo with Velocity Motion Model and Likelyhood Field
    MCL mcl(800, "vel", "likelyhood");

    while(ros::ok()) {
        ros::spin();
    }
    return 0;
}