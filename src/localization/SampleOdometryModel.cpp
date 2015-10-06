#include "SampleOdometryModel.hpp"

SampleOdometryModel::SampleOdometryModel() : SampleMotionModel(), cmd("/odom", 1000) {}

void SampleOdometryModel::samplePose2D(Pose2D *pose) {
    
}