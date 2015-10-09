#ifndef BEAM_RANGE_FINDER_MODEL_H
#define BEAM_RANGE_FINDER_MODEL_H

#include "MeasurementModel.hpp"

class BeamRangeFinderModel : public MeasurementModel {
    public:
        BeamRangeFinderModel(Laser*, Map*);
        // base class abstract method implementation
        virtual void getWeight(Pose2D *);
        // set the Laser
        virtual void setLaser(Laser *);


};

#endif