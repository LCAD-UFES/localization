#include "LikelyhoodFieldModel.hpp"

// Basic constructor
LikelyhoodFieldModel::LikelyhoodFieldModel(Laser *ls, Map *m) : MeasurementModel(ls, m) {}

// assigns a weight to to all particles/samples
void LikelyhoodFieldModel::getWeight(double *w) {
    *w = 0.025;
}

// get the map pointer
Map* LikelyhoodFieldModel::getMap() {
    return map;
}

