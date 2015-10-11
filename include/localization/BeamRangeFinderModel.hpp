#ifndef BEAM_RANGE_FINDER_MODEL_H
#define BEAM_RANGE_FINDER_MODEL_H

#include "MeasurementModel.hpp"

class BeamRangeFinderModel : public MeasurementModel {
    public:
        // the beam model
        BeamRangeFinderModel(Laser*, Map*);
        // base class abstract method implementation
        virtual void getWeights(SampleSet *);
};

#endif