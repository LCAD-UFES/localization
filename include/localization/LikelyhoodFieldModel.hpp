#ifndef LIKELYHOOD_FIELD_MODEL_H
#define LIKELYHOOD_FIELD_MODEL_H

#include "MeasurementModel.hpp"

class LikelyhoodFieldModel : public MeasurementModel {

    public:
        // basic constructor
        LikelyhoodFieldModel(Laser *, Map *);
        // base class abstract method implementation
        virtual void getWeight(double*);
        // get the map pointer
        virtual Map* getMap();

};

#endif