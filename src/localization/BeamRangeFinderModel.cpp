#include "BeamRangeFinderModel.hpp"

// basic constructor
BeamRangeFinderModel::BeamRangeFinderModel(Laser *ls, Map *m) : MeasurementModel(ls, m) {}

//
void BeamRangeFinderModel::getWeight(Sample2D *sample) {
    /* TODO */
}

// get the map pointer
Map* BeamRangeFinderModel::getMap() {
    return map;
}

// update the LaserScan
ros::Time BeamRangeFinderModel::updateLaser() {
    /* TODO */
    return ros::Time::now();
}