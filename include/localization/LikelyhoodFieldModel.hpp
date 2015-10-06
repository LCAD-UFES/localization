#ifndef LIKELYHOOD_FIELD_MODEL_H
#define LIKELYHOOD_FIELD_MODEL_H

#include "MeasurementModel.hpp"

class LikelyhoodFieldModel : public MeasurementModel {
    public:
        // base class abstract method implementation
        virtual double getWeight(Pose2D *);

};

#endif