#include "LikelyhoodFieldModel.hpp"

// Basic constructor
LikelyhoodFieldModel::LikelyhoodFieldModel(Laser *ls, Map *m) : MeasurementModel(ls, m) {}

void LikelyhoodFieldModel::getWeight(Pose2D *pose) {}