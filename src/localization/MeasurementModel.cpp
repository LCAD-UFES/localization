#include "MeasurementModel.hpp"

// basic constructor
MeasurementModel::MeasurementModel(Laser *ls, Map *m) : laser(ls), map(m) {}

// the laser must be deleted by the ParticleFilter object
// not here
MeasurementModel::~MeasurementModel() {
    laser = nullptr;
    map = nullptr;
}