#include "BeamRangeFinderModel.hpp"

// basic constructor
BeamRangeFinderModel::BeamRangeFinderModel(ros::NodeHandle &private_nh, Laser *ls, Map *m) : MeasurementModel(ls, m) {}

//
double BeamRangeFinderModel::getWeight(Sample2D *sample) {
    /* TODO */
}

// update the LaserScan
ros::Time BeamRangeFinderModel::update() {
    /* TODO */
    return ros::Time::now();
}