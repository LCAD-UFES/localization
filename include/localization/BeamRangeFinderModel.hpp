#ifndef BEAM_RANGE_FINDER_MODEL_H
#define BEAM_RANGE_FINDER_MODEL_H

#include <ros/ros.h>

#include "MeasurementModel.hpp"
// #include "occupancy_grid_utils/ray_tracer.h"


class BeamRangeFinderModel : public MeasurementModel {

    private:
        //BeamRangeFinderModel parameters
        double z_hit, z_short, z_max, z_rand, sigma_hit, lambda_short;
        //aux
        double ztk, ztk_real, delta_ztk, part_gaussian, z_random_max, sigma_hit2;

    public:
        // the beam model
        BeamRangeFinderModel(ros::NodeHandle&, Laser*, Map* );
        // base class abstract method implementation
        virtual double getWeight(Sample2D *);
        // update the LaserScan
        virtual ros::Time update();

};

#endif
