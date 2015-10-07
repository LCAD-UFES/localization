#ifndef SAMPLE_MOTION_MODEL_H
#define SAMPLE_MOTION_MODEL_H

#include "Pose2D.hpp"

class SampleMotionModel {
    public:
        // sample a new pose from a given command and previous pose
        // abstract method, so it's an abstract class
        virtual void samplePose2D(Pose2D *) =0;
};

#endif