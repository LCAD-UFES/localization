#include "BeamRangeFinderModel.hpp"

// basic constructor
BeamRangeFinderModel::BeamRangeFinderModel(Laser *ls, Map *m) : MeasurementModel(ls, m) {}

//
void BeamRangeFinderModel::getWeights(SampleSet *Xt) {}