#include "MonteCarloLocalization.hpp"

// this constructor receives the string parameters
MCL::MCL(unsigned int s, std::string vm, string mm) : Xt(s, vm, mm) {}

// this one receives the models
MCL::MCL(unsigned int s, SampleMotionModel *motionM, MeasurementModel *measurementM) : Xt(s, motionM, measurementM) {}


