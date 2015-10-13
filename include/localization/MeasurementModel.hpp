#ifndef SAMPLE_MEASUREMENT_MODEL_H
#define SAMPLE_MEASUREMENT_MODEL_H

#include <mutex>

#include "Map.hpp"
#include "Laser.hpp"
#include "Sample2D.hpp"

class MeasurementModel {
    protected:

        // a pointer to the global Laser
        Laser *laser;

        // our internal LaserScan representation
        LS_60 ls_scan;

        // the Map object contains the OccupancyGrid
        Map *map;

    public:

        MeasurementModel(Laser*, Map *);
        // the laser must be deleted by the ParticleFilter object
        // not here
        ~MeasurementModel();
        // abstract getWeight method
        virtual void getWeight(Sample2D*) =0;
        // get the map pointer
        virtual Map* getMap() =0;
        // update the LaserScan
        virtual ros::Time updateLaser() =0;
};

#endif