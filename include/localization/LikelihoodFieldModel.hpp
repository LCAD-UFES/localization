#ifndef Likelihood_FIELD_MODEL_H
#define Likelihood_FIELD_MODEL_H

#include "MeasurementModel.hpp"

class LikelihoodFieldModel : public MeasurementModel {

    public:
        // basic constructor
        LikelihoodFieldModel(Laser*, Map *);

        // base class abstract method implementation
        virtual void getWeight(double*);

        // get the map pointer
        virtual Map* getMap();

        // update the LaserScan
        virtual ros::Time updateLaser();

};

#endif