#include "LikelihoodFieldModel.hpp"

// Basic constructor
LikelihoodFieldModel::LikelihoodFieldModel(Laser *ls, Map *m) : MeasurementModel(ls, m) {}

// assigns a weight to to all particles/samples
void LikelihoodFieldModel::getWeight(Sample2D *sample) {

    // auxiliar variables
    double q = 1.0;
    double dist;

    // iterate over the scans
    // we have 60 
    for (int i = 0; i < ls; i++) {
    }

    //
    sample->weight = 0.025;
}

// get the map pointer
Map* LikelihoodFieldModel::getMap() {
    return map;
}

// update the LaserScan
ros::Time LikelihoodFieldModel::updateLaser() {
    // update the protected LS_60 - see MeasurementModel base class
    // the Laser object manages the mutex
    laser->getScan(&ls_scan);

    return ls_scan.time;
}