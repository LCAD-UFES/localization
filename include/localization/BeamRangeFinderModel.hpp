#ifndef BEAM_RANGE_FINDER_MODEL_H
#define BEAM_RANGE_FINDER_MODEL_H

#include "MeasurementModel.hpp"

class BeamRangeFinderModel : public MeasurementModel {
    public:
        // the beam model
        BeamRangeFinderModel(Laser *, Map *);
        // base class abstract method implementation
        virtual void getWeight(Sample2D *);
        // get the map pointer
        virtual Map* getMap();
        // update the LaserScan
        virtual ros::Time updateLaser();
};

#endif