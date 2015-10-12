#include "BeamRangeFinderModel.hpp"

// basic constructor
BeamRangeFinderModel::BeamRangeFinderModel(Laser *ls, Map *m) : MeasurementModel(ls, m) {}

//
void BeamRangeFinderModel::getWeight(double *w) {
    /* TODO */
}

// get the map pointer
Map* BeamRangeFinderModel::getMap() {
    return map;
}