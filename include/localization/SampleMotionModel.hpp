#ifndef SAMPLE_MOTION_MODEL_H
#define SAMPLE_MOTION_MODEL_H

#include "GaussianPDF.hpp"
#include "Pose2D.hpp"
#include "CommandReader.hpp"


class SampleMotionModel {
    protected:
        // the PDF object
        GaussianPDF sampler;

    public:
        // sample a new pose from a given command and previous pose
        // abstract method, so it's an abstract class
        virtual void samplePose2D(Pose2D *) =0;
};

#endif