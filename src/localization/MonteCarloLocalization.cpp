#include "MonteCarloLocalization.hpp"

// this constructor receives the amount of samples
// it needs to specify the velocity and measurement models
// Velocity Models: "vel" == "velocity" or "odom" == odometry
// Measurement Models: "beam" == beam range find or "likelyhood" == likelyhood field
// and finally the map filename as the last argument
MCL::MCL(unsigned int s, std::string motion, std::string measurement) : nh(), Xt(s, motion, measurement) {
    
}

// this one receives the models
MCL::MCL(unsigned int s, SampleMotionModel *motionM, MeasurementModel *measurementM) : nh(), Xt(s, motionM, measurementM) {}


