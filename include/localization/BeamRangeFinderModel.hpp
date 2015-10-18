#ifndef BEAM_RANGE_FINDER_MODEL_H
#define BEAM_RANGE_FINDER_MODEL_H

#include <ros/ros.h>

#include "MeasurementModel.hpp"

class BeamRangeFinderModel : public MeasurementModel {
    public:
        // the beam model
        BeamRangeFinderModel(ros::NodeHandle&, Laser *, Map *);
        // base class abstract method implementation
        virtual double getWeight(Sample2D *);
        // update the LaserScan
        virtual ros::Time update();
        int t;
};

#endif