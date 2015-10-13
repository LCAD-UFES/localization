#ifndef Likelihood_FIELD_MODEL_H
#define Likelihood_FIELD_MODEL_H

#include <ros/ros.h>

#include "MeasurementModel.hpp"


class LikelihoodFieldModel : public MeasurementModel {

    private:
        // LikelihoodFieldModel parameters
        double z_hit, z_max, z_rand;

    public:
        // basic constructor
        LikelihoodFieldModel(ros::NodeHandle&, Laser*, Map*);

        // base class abstract method implementation
        virtual void getWeight(Sample2D *);

        // get the map pointer
        virtual Map* getMap();

        // update the LaserScan
        virtual ros::Time updateLaser();

};

#endif