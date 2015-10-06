#ifndef SAMPLE_MEASUREMENT_MODEL_H
#define SAMPLE_MEASUREMENT_MODEL_H

#include "Pose2D.hpp"

class MeasurementModel {
    // it needs to assign to the correct topics
    //
    public:
        // abstract method
        virtual double getWeight(Pose2D *) =0;
};

#endif