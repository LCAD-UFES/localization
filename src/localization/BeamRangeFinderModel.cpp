#include "BeamRangeFinderModel.hpp"

// basic constructor
BeamRangeFinderModel(Laser *ls, Map *m) : MeasurementModel(ls, m) {}


void BeamRangeFinderModel::getWeight(Pose2D *pose) {}