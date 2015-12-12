#ifndef BEAM_RANGE_FINDER_MODEL_H
#define BEAM_RANGE_FINDER_MODEL_H

#include <ros/ros.h>
#include "MeasurementModel.hpp"

class BeamRangeFinderModel : public MeasurementModel {

    private:
        //BeamRangeFinderModel parameters and aux
        double z_hit, z_short, z_max, z_rand, sigma_hit, lambda_short, sigma_den, p_rand;

        // computes the probability
        inline double prob(float, float);

    public:
        // the beam model
        BeamRangeFinderModel(ros::NodeHandle&, Laser*, Map* );

        // base class abstract method implementation
        virtual double getWeight(Sample2D *);

        // update the LaserScan
        virtual ros::Time update();

};


#endif
