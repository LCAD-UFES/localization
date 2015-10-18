#include "ParticleFilter.hpp"

int main(int argc, char **argv) {
    // ROS
    ros::init(argc, argv, "localization");

    ParticleFilter pf;

    pf.start();

    return 0;
}
