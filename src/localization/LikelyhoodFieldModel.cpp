#include "LikelyhoodFieldModel.hpp"

double LikelyhoodFieldModel::getWeight(Pose2D *pose) {
    return 0.0;
}
void LikelyhoodFieldModel::setLaser(Laser *l) {
    laser = l;
}