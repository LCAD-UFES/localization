#ifndef LIKELYHOOD_FIELD_MODEL_H
#define LIKELYHOOD_FIELD_MODEL_H

#include "MeasurementModel.hpp"
#include "SampleSet.hpp"

class LikelyhoodFieldModel : public MeasurementModel {

    public:
        // basic constructor
        LikelyhoodFieldModel(Laser *, Map *);
        // base class abstract method implementation
        virtual void getWeights(SampleSet *);

};

#endif