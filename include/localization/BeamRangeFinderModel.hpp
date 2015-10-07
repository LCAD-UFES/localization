#ifndef BEAM_RANGE_FINDER_MODEL_H
#define BEAM_RANGE_FINDER_MODEL_H

#include "MeasurementModel.hpp"

class BeamRangeFinderModel : public MeasurementModel {
    private:
        // the subscriber to the map server
        ros::Subscriber sub;
    public:
        // base class abstract method implementation
        virtual double getWeight(Pose2D *);
        // set the Laser
        virtual void setLaser(Laser *);
        
        // the callback method to the map server
        /* TODO */

};

#endif