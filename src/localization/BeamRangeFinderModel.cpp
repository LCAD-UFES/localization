#include "BeamRangeFinderModel.hpp"

double BeamRangeFinderModel::getWeight(Pose2D *pose) {
    return 0.0;
}

void BeamRangeFinderModel::setLaser(Laser *l) {
    laser = l;
}