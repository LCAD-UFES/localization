#include "MeasurementModel.hpp"

// basic constructor
MeasurementModel::MeasurementModel(Laser* ls, Map *m) : laser(ls), ls_scan(), step(1), map(m) {}

// the laser must be deleted by the ParticleFilter object
// not here
MeasurementModel::~MeasurementModel() {
    map = nullptr;
}

// get the map pointer
Map* MeasurementModel::getMap() {
    return map;
}