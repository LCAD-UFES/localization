#ifndef SAMPLE_MEASUREMENT_MODEL_H
#define SAMPLE_MEASUREMENT_MODEL_H

#include "Pose2D.hpp"
#include "Laser.hpp"

class MeasurementModel {
    private:
        // The Laser object maintains a PointCloud
        Laser laser*;
    public:
        // the laser must be deleted by the MCL object
        ~MeasurementModel() { laser = nullptr; }
        // abstract getWeight method
        virtual double getWeight(Pose2D *) =0;
        // set the Laser, it's also an abstract method
        virtual void setLaser(Laser *) =0;
};

#endif