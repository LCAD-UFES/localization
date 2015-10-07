#ifndef SAMPLE_MOTION_MODEL_H
#define SAMPLE_MOTION_MODEL_H

#include "Pose2D.hpp"

class SampleMotionModel {
    // the random generator
    std::default_random_engine generator;
    public:
        SampleMotionModel();
        // sample a new pose from a given command and previous pose
        // abstract method, so it's an abstract class
        virtual void samplePose2D(Pose2D *) =0;
        // returns a double from a normal distribution PDF
        // the mean is zero centered and the it receives the variance as input
        // inside the method we get the standard deviation
        virtual double gaussianPDF(double var);

};

#endif