#ifndef BEAM_RANGE_FINDER_MODEL_H
#define BEAM_RANGE_FINDER_MODEL_H

#include "MeasurementModel.hpp"

class BeamRangeFinderModel : public MeasurementModel {
    public:
        // base class abstract method implementation
        virtual double getWeight(Pose2D *);

};

#endif