#include "BeamRangeFinderModel.hpp"

// basic constructor
BeamRangeFinderModel::BeamRangeFinderModel(ros::NodeHandle &private_nh, Laser *ls, Map *m) : MeasurementModel(ls, m) {}

//
void BeamRangeFinderModel::getWeight(Sample2D *sample) {
    /* TODO */
}

// get the map pointer
Map* BeamRangeFinderModel::getMap() {
    return map;
}

// update the LaserScan
ros::Time BeamRangeFinderModel::update() {
    /* TODO */
    return ros::Time::now();
}